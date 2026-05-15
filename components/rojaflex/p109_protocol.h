// p109_protocol.h
// Rojaflex P109 protocol only (no ESPHome / CC1101 includes).
// Pure C++ protocol layer, no transport dependencies.
// Used by rojaflex.cpp (Hub) and tested standalone via test_rojaflex_crc_commands.py.

#pragma once

#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

namespace rojaflex {

// Number of physical channels the P109 protocol can address (4-bit nibble).
inline constexpr uint8_t MAX_CHANNELS = 16;

// Number of identical consecutive frames that must be observed before a
// candidate housecode is auto-accepted.
inline constexpr uint32_t AUTO_LEARN_FRAMES_REQUIRED = 3;

// Sentinel value that means "no housecode configured yet" / "auto-learn enabled".
inline constexpr const char *UNCONFIGURED_HOUSECODE = "0000000";

// P109 protocol command codes (as transmitted in the high nibble of payload[5]).
enum class Command : uint8_t {
  STOP = 0x0,
  UP = 0x1,
  DOWN = 0x8,
  SAVE_FAV = 0x9,
  GOTO_FAV = 0xD,
  REQUEST = 0xE,
};

/// Validate a housecode string: exactly 7 hex characters.
inline bool is_valid_housecode(const std::string &housecode) {
  if (housecode.length() != 7) return false;
  for (char c : housecode) {
    if (!std::isxdigit(static_cast<unsigned char>(c))) return false;
  }
  return true;
}

/// Housecode is configured when it is not empty and not the default placeholder.
inline bool is_housecode_configured(const std::string &housecode) {
  return !housecode.empty() && housecode != UNCONFIGURED_HOUSECODE;
}

/// Manual housecode from API/service: 7 hex chars, resets auto-learn state.
inline bool apply_manual_housecode(const std::string &new_housecode, std::string &housecode,
                                   std::string &auto_learn_housecode, uint32_t &auto_learn_count) {
  if (!is_valid_housecode(new_housecode)) return false;
  housecode = new_housecode;
  auto_learn_housecode.clear();
  auto_learn_count = 0;
  return true;
}

// Note on integrity checking: the 9th byte of every P109 payload is an 8-bit
// sum of bytes 1..7. We do NOT verify it on RX because the CC1101 already
// validates a 16-bit hardware CRC (CRC_EN=1) and drops corrupted frames.
// The hardware CRC is strictly stronger than the 8-bit sum.
inline bool is_valid_p109_payload(const std::vector<uint8_t> &payload) {
  return payload.size() == 9 && payload[0] == 0x08;
}

inline std::string extract_housecode_from_payload(const std::vector<uint8_t> &payload) {
  if (!is_valid_p109_payload(payload)) return "";
  char rx_hc_buf[8];
  snprintf(rx_hc_buf, sizeof(rx_hc_buf), "%02X%02X%02X%1X", payload[1], payload[2], payload[3],
           (payload[4] >> 4) & 0x0F);
  return std::string(rx_hc_buf);
}

struct AutoLearnResult {
  bool learned_now{false};
  std::string configured_housecode;
  std::string candidate_housecode;
  uint32_t candidate_count{0};
};

inline AutoLearnResult auto_learn_housecode_step(const std::vector<uint8_t> &payload,
                                                 const std::string &configured_housecode,
                                                 const std::string &candidate_housecode,
                                                 uint32_t candidate_count) {
  AutoLearnResult result;
  result.configured_housecode = configured_housecode;
  result.candidate_housecode = candidate_housecode;
  result.candidate_count = candidate_count;

  if (!is_valid_p109_payload(payload) || is_housecode_configured(configured_housecode)) {
    return result;
  }

  const std::string rx_housecode = extract_housecode_from_payload(payload);
  if (rx_housecode.empty()) return result;

  if (rx_housecode == candidate_housecode) {
    result.candidate_count = candidate_count + 1;
    if (result.candidate_count >= AUTO_LEARN_FRAMES_REQUIRED) {
      result.learned_now = true;
      result.configured_housecode = rx_housecode;
      result.candidate_housecode = "";
      result.candidate_count = 0;
    }
    return result;
  }

  result.candidate_housecode = rx_housecode;
  result.candidate_count = 1;
  return result;
}

// Source of a position update derived from a P109 frame.
enum class PositionSource : uint8_t {
  None,           // frame valid but no position info (STOP/SAVE/GOTO/REQUEST/...)
  MotorFeedback,  // DEV=5: exact percentage from tubular motor
  RemoteInferred, // DEV=A + UP/DOWN: 0%/100% inferred from remote command
};

/// Unified P109 frame decoder.
struct P109Frame {
  bool valid{false};
  bool housecode_match{false};
  std::string rx_housecode;
  uint8_t channel{0};
  // channel==0 means "all channels" only for remote frames (DEV=A).
  bool applies_to_all_channels{false};
  uint8_t device_type{0};
  uint8_t cmd{0};
  uint8_t cmd_value{0};
  PositionSource position_source{PositionSource::None};
  int pct{0};  // only meaningful if position_source != None
  std::string raw;
  std::string info;
};

inline P109Frame decode_p109_frame(const std::vector<uint8_t> &payload,
                                   const std::string &configured_housecode) {
  P109Frame f;
  if (!is_valid_p109_payload(payload)) return f;
  f.valid = true;

  f.rx_housecode = extract_housecode_from_payload(payload);
  f.channel = payload[4] & 0x0F;
  const uint8_t cmd_dev = payload[5];
  f.device_type = cmd_dev & 0x0F;
  f.cmd = (cmd_dev >> 4) & 0x0F;
  f.cmd_value = payload[6];
  f.applies_to_all_channels = (f.device_type == 0xA && f.channel == 0);

  char raw_buf[32];
  snprintf(raw_buf, sizeof(raw_buf), "%02X%02X%02X%02X%02X%02X%02X%02X%02X", payload[0], payload[1],
           payload[2], payload[3], payload[4], payload[5], payload[6], payload[7], payload[8]);
  f.raw = std::string(raw_buf);

  char info_buf[96];
  snprintf(info_buf, sizeof(info_buf), "HC=%s CH=%u DEV=%X CMD=%X VAL=%u", f.rx_housecode.c_str(),
           f.channel, f.device_type, f.cmd, f.cmd_value);
  f.info = std::string(info_buf);

  if (!is_housecode_configured(configured_housecode) || f.rx_housecode != configured_housecode) {
    return f;
  }
  f.housecode_match = true;

  if (f.device_type == 0x5) {
    f.position_source = PositionSource::MotorFeedback;
    f.pct = f.cmd_value > 100 ? 100 : f.cmd_value;
  } else if (f.device_type == 0xA) {
    if (f.cmd == 0x1) {
      f.position_source = PositionSource::RemoteInferred;
      f.pct = 0;
    } else if (f.cmd == 0x8) {
      f.position_source = PositionSource::RemoteInferred;
      f.pct = 100;
    }
  }

  return f;
}

// Software emulation of "drive to position X%". Position semantics: 0=open, 100=closed.
// ESPHome's cover convention (0.0=closed, 1.0=open) must be inverted by the caller.
struct ShutterMotionPlan {
  enum class Action : uint8_t {
    None,         // do nothing (e.g. mid-position without known current pos)
    Stop,         // already at target, only emit a STOP frame
    UpToEnd,      // drive UP to the end stop (no timed STOP required)
    DownToEnd,    // drive DOWN to the end stop (no timed STOP required)
    UpThenStop,   // drive UP, schedule STOP after duration_ms
    DownThenStop, // drive DOWN, schedule STOP after duration_ms
  };
  Action action{Action::None};
  uint32_t duration_ms{0};
  int target_pct{0};
  const char *info{""};
};

inline ShutterMotionPlan compute_shutter_motion_plan(int current_pct, int target_pct,
                                                     int time_to_open_s, int time_to_close_s) {
  ShutterMotionPlan plan;

  if (target_pct < 0) target_pct = 0;
  if (target_pct > 100) target_pct = 100;
  plan.target_pct = target_pct;

  if (target_pct == 0) {
    plan.action = ShutterMotionPlan::Action::UpToEnd;
    return plan;
  }
  if (target_pct == 100) {
    plan.action = ShutterMotionPlan::Action::DownToEnd;
    return plan;
  }

  if (current_pct < 0) {
    plan.info = "current position unknown - move to an end stop first";
    return plan;
  }

  if (current_pct == target_pct) {
    plan.action = ShutterMotionPlan::Action::Stop;
    return plan;
  }

  if (target_pct > current_pct) {
    if (time_to_close_s < 0) {
      plan.info = "close time not calibrated - drive fully closed once";
      return plan;
    }
    const uint32_t delta = static_cast<uint32_t>(target_pct - current_pct);
    plan.duration_ms = (delta * static_cast<uint32_t>(time_to_close_s) * 1000u) / 100u;
    plan.action = ShutterMotionPlan::Action::DownThenStop;
  } else {
    if (time_to_open_s < 0) {
      plan.info = "open time not calibrated - drive fully open once";
      return plan;
    }
    const uint32_t delta = static_cast<uint32_t>(current_pct - target_pct);
    plan.duration_ms = (delta * static_cast<uint32_t>(time_to_open_s) * 1000u) / 100u;
    plan.action = ShutterMotionPlan::Action::UpThenStop;
  }
  return plan;
}

inline bool build_tx_packet(const std::string &housecode, uint8_t channel, uint8_t cmd_code,
                            std::vector<uint8_t> &tx_packet, std::string &final_msg) {
  if (!is_housecode_configured(housecode)) return false;

  // Decode the 7-hex-char housecode into the upper 28 bits of payload bytes 1..4.
  const uint32_t hc28 = static_cast<uint32_t>(strtoul(housecode.c_str(), nullptr, 16));

  const uint8_t cmd_nib = cmd_code & 0x0F;
  const uint8_t cmd_dev = static_cast<uint8_t>((cmd_nib << 4) | 0xA);  // DEV=A (remote)

  tx_packet.clear();
  tx_packet.resize(9);
  tx_packet[0] = 0x08;
  tx_packet[1] = static_cast<uint8_t>((hc28 >> 20) & 0xFF);
  tx_packet[2] = static_cast<uint8_t>((hc28 >> 12) & 0xFF);
  tx_packet[3] = static_cast<uint8_t>((hc28 >> 4) & 0xFF);
  tx_packet[4] = static_cast<uint8_t>(((hc28 & 0x0F) << 4) | (channel & 0x0F));
  tx_packet[5] = cmd_dev;
  tx_packet[6] = 0x01;
  tx_packet[7] = cmd_dev;

  uint8_t sum = 0;
  for (size_t i = 1; i <= 7; i++) sum += tx_packet[i];
  tx_packet[8] = sum;

  char buf[32];
  snprintf(buf, sizeof(buf), "P109#%02X%02X%02X%02X%02X%02X%02X%02X%02X",
           tx_packet[0], tx_packet[1], tx_packet[2], tx_packet[3], tx_packet[4],
           tx_packet[5], tx_packet[6], tx_packet[7], tx_packet[8]);
  final_msg = buf;
  return true;
}

}  // namespace rojaflex
