// rojaflex.h
// Hub class for the Rojaflex ESPHome external component.
// Implements CC1101Listener and orchestrates RX routing, AutoLearn,
// per-channel calibration, position interpolation, and TX.
// Cover / sensor sub-platforms register themselves against this hub.

#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "esphome/components/cc1101/cc1101.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/application.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "esphome/core/preferences.h"

#include "p109_protocol.h"

namespace rojaflex {

// Pull esphome's time/delay primitives into our namespace.
// Without these, esphome's Arduino-compat globals are not available under
// esp-idf builds and `millis()` / `delay()` fail to compile.
using esphome::delay;
using esphome::millis;

class RojaflexCover;  // forward declaration

// NVS storage layouts (fixed-size to work with ESPPreferenceObject).
struct HousecodeStorage {
  char value[8];  // 7 hex chars + null terminator
};
struct CalStorage {
  char value[256];  // serialized "ch:open,close;..." string
};

class RojaflexHub : public esphome::Component, public esphome::cc1101::CC1101Listener {
 public:
  // ── Configuration setters (called from Python-generated code) ───────────

  void set_cc1101_parent(esphome::cc1101::CC1101Component *cc1101) { cc1101_ = cc1101; }
  void set_housecode(const std::string &housecode) { housecode_ = housecode; }
  void set_tx_repetitions(uint8_t reps) { tx_repetitions_ = reps; }

  // ── Sensor registration (called from sub-platform to_code) ──────────────

  void set_housecode_sensor(esphome::text_sensor::TextSensor *s) { housecode_sensor_ = s; }
  void set_last_rx_info_sensor(esphome::text_sensor::TextSensor *s) { last_rx_info_sensor_ = s; }
  void set_last_tx_sensor(esphome::text_sensor::TextSensor *s) { last_tx_sensor_ = s; }

  // Cover registration
  void register_cover(RojaflexCover *cover);

  // ── Component lifecycle ─────────────────────────────────────────────────

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return esphome::setup_priority::DATA; }

  // ── CC1101Listener ──────────────────────────────────────────────────────

  void on_packet(const std::vector<uint8_t> &packet, float freq_offset, float rssi,
                 uint8_t lqi) override;

  // ── Public accessors (used by covers and text sensors) ──────────────────

  int get_motor_pct(uint8_t channel) const;
  int get_cal_time_open_s(uint8_t channel) const;
  int get_cal_time_close_s(uint8_t channel) const;
  const std::string &get_housecode() const { return housecode_; }
  const std::string &get_auto_learn_housecode() const { return auto_learn_housecode_; }
  uint32_t get_auto_learn_count() const { return auto_learn_count_; }
  const std::string &get_last_rx_info() const { return last_rx_info_; }

  // ── Actions called by covers ─────────────────────────────────────────────

  // Send UP/DOWN/STOP command with full calibration-capture and interpolation logic.
  void send_command(uint32_t channel_id, uint8_t cmd_code);

  // Drive to target_pct (0=open, 100=closed). Uses calibrated times for mid-pos.
  void set_position(uint32_t channel_id, int target_pct);

  // ── API actions (from YAML services) ────────────────────────────────────

  bool apply_manual_housecode(const std::string &new_housecode);

  // Called by RojaflexCover on every HA-dashboard STOP. Tracks a per-channel
  // gesture (5 stops within 2 s) that resets only that channel's calibration —
  // replaces the global reset_calibration_times button. Physical-remote STOPs
  // never enter here, so panic-mashing the remote can't accidentally wipe cal.
  void note_ha_stop(uint8_t channel);

  // ── Status string helpers (used by per-cover text sensor) ───────────────

  std::string get_channel_status_string(uint8_t channel) const;

 protected:
  // ── Persistence helpers ─────────────────────────────────────────────────

  void persist_housecode_();
  void load_housecode_();
  void persist_calibration_();
  void load_calibration_();
  std::string serialize_calibration_() const;
  void deserialize_calibration_(const std::string &s);

  // ── Calibration capture helpers ─────────────────────────────────────────

  void cancel_calibration_capture_(uint32_t ch);
  void arm_calibration_capture_(uint32_t ch, int target_pct);
  bool try_resolve_calibration_(uint32_t ch, int reported_pct);

  // ── Position interpolation helpers ──────────────────────────────────────

  void cancel_position_interpolation_(uint32_t ch);
  void arm_position_interpolation_(uint32_t ch, int start_pct, int end_pct, uint32_t duration_ms);
  void tick_position_interpolation_();

  // ── Mid-position scheduler helpers ──────────────────────────────────────

  void cancel_mid_position_timers_(uint8_t channel, bool all_channels);
  std::string mid_position_timer_name_(uint32_t channel_id) const;

  // ── Internal TX ─────────────────────────────────────────────────────────

  // Core send helper; optimistic_update=false for mid-position initial drive.
  void send_command_(uint32_t channel_id, uint8_t cmd_code, bool optimistic_update);

  // Transmit packet via cc1101_, update last_self_tx_ms_. Returns the
  // CC1101 driver result so send_command can surface the specific error
  // (TIMEOUT / PARAMS / PLL_LOCK) in the last_tx text sensor.
  esphome::cc1101::CC1101Error transmit_(const std::vector<uint8_t> &packet);

  // ── Sensor publishing ───────────────────────────────────────────────────

  void publish_housecode_sensor_();

  // ── Per-channel last-command tracking ───────────────────────────────────
  void update_last_cmd_(const P109Frame &frame, float rssi);
  void set_last_cmd_(uint8_t ch, const char *name, float rssi);
  // Push updated status string to covers whose channel matches (or all if broadcast).
  void notify_covers_status_(uint8_t ch, bool all_channels);

 private:
  esphome::cc1101::CC1101Component *cc1101_{nullptr};

  std::string housecode_;
  std::string auto_learn_housecode_;
  uint32_t auto_learn_count_{0};
  uint8_t tx_repetitions_{2};

  std::vector<int> motor_pct_;  // per-channel, -1 = unknown

  // Calibration state
  std::vector<int> cal_time_open_s_;
  std::vector<int> cal_time_close_s_;
  std::vector<uint32_t> cal_pending_tx_ms_;
  std::vector<int> cal_pending_target_pct_;
  uint32_t last_self_tx_ms_{0};

  // Per-channel sliding window of HA-dashboard STOP timestamps. Filling the
  // window (5 stops within 2 s on the same channel) wipes that channel's
  // calibration — gesture-based replacement for the diagnostic reset button.
  static constexpr size_t HA_STOP_RESET_PRESSES = 5;
  static constexpr uint32_t HA_STOP_RESET_WINDOW_MS = 2000;
  std::vector<std::array<uint32_t, HA_STOP_RESET_PRESSES>> ha_stop_ring_;

  // Interpolation state
  std::vector<uint32_t> interp_start_ms_;
  std::vector<int> interp_start_pct_;
  std::vector<int> interp_end_pct_;
  std::vector<uint32_t> interp_duration_ms_;

  // Sensor state
  std::string last_rx_info_{"-"};

  // Per-channel: name of last received command (any DEV type) + its RSSI.
  // "" means no frame received yet on this channel. Set before PositionSource filter
  // so STOP / SAVE / motor-feedback all appear in the status sensor.
  std::vector<std::string> last_cmd_name_;
  std::vector<float> last_cmd_rssi_;

  // Optional sensors (nullptr if not declared in YAML)
  esphome::text_sensor::TextSensor *housecode_sensor_{nullptr};
  esphome::text_sensor::TextSensor *last_rx_info_sensor_{nullptr};
  esphome::text_sensor::TextSensor *last_tx_sensor_{nullptr};

  // Registered covers (notified on motor_pct_ changes via get_motor_pct())
  std::vector<RojaflexCover *> covers_;

  // NVS preferences
  esphome::ESPPreferenceObject housecode_pref_;
  esphome::ESPPreferenceObject cal_state_pref_;

  // Timing for loop() throttles
  uint32_t last_interp_tick_ms_{0};
  uint32_t last_status_publish_ms_{0};

  // Calibration constants
  static constexpr uint32_t CALIBRATION_CAPTURE_WINDOW_MS = 60'000;
  static constexpr int CALIBRATION_MIN_TIME_S = 5;
  static constexpr uint32_t SELF_TX_ECHO_GUARD_MS = 200;
};

// Generic vector-resize helper used in both Hub and sub-platforms.
template <typename T>
inline void ensure_capacity(std::vector<T> &vec, size_t min_size, T default_value) {
  if (vec.size() < min_size) vec.resize(min_size, default_value);
}

}  // namespace rojaflex
