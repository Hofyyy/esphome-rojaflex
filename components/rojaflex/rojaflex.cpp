// rojaflex.cpp
// Hub implementation for the Rojaflex external component. Owns RX routing,
// AutoLearn, per-channel calibration, position interpolation, and TX via
// the native ESPHome cc1101 component.

#include "rojaflex.h"
#include "cover/rojaflex_cover.h"

#include <cstdio>
#include <algorithm>

namespace rojaflex {

static const char *const TAG_HUB = "rojaflex";

// ─────────────────────────────────────────────────────────────────────────────
// Component lifecycle
// ─────────────────────────────────────────────────────────────────────────────

void RojaflexHub::setup() {
  motor_pct_.assign(MAX_CHANNELS, -1);
  ha_stop_ring_.assign(MAX_CHANNELS, std::array<uint32_t, HA_STOP_RESET_PRESSES>{});

  // Load persisted housecode from NVS (may override the config default).
  load_housecode_();

  // Load persisted calibration.
  load_calibration_();

  // Register ourselves as a CC1101 packet listener.
  if (cc1101_ != nullptr) {
    cc1101_->register_listener(this);
  } else {
    ESP_LOGE(TAG_HUB, "cc1101 parent not set — hub will not receive packets");
    this->mark_failed();
  }
}

void RojaflexHub::loop() {
  const uint32_t now = millis();

  // Interpolation tick every 500 ms (animates mid-position and calibrated
  // end-stop runs). Only updates motor_pct_ entries that have an armed
  // interpolation; covers pick up the change via get_motor_pct().
  if (now - last_interp_tick_ms_ >= 500) {
    last_interp_tick_ms_ = now;
    tick_position_interpolation_();
  }

  // Combined housecode/status sensor every 2 s.
  if (now - last_status_publish_ms_ >= 2000) {
    last_status_publish_ms_ = now;
    publish_housecode_sensor_();
  }
}

void RojaflexHub::dump_config() {
  ESP_LOGCONFIG(TAG_HUB, "Rojaflex Hub:");
  ESP_LOGCONFIG(TAG_HUB, "  Housecode: %s", housecode_.c_str());
  ESP_LOGCONFIG(TAG_HUB, "  TX repetitions: %u", static_cast<unsigned>(tx_repetitions_));
}

// ─────────────────────────────────────────────────────────────────────────────
// CC1101Listener — RX dispatch
// ─────────────────────────────────────────────────────────────────────────────

void RojaflexHub::on_packet(const std::vector<uint8_t> &packet, float freq_offset, float rssi,
                             uint8_t lqi) {
  // Phase 1: validate frame shape.
  if (!is_valid_p109_payload(packet)) {
    return;
  }

  // Phase 2: while no housecode is known, only feed the auto-learn state machine.
  if (!is_housecode_configured(housecode_)) {
    auto learn = auto_learn_housecode_step(packet, housecode_, auto_learn_housecode_,
                                           auto_learn_count_);
    housecode_ = learn.configured_housecode;
    auto_learn_housecode_ = learn.candidate_housecode;
    auto_learn_count_ = learn.candidate_count;

    if (learn.learned_now) {
      ESP_LOGI(TAG_HUB, "AUTO-LEARNED housecode: %s", housecode_.c_str());
      persist_housecode_();
    } else if (!auto_learn_housecode_.empty()) {
      ESP_LOGD(TAG_HUB, "Learning candidate housecode: %s (%u/%u)",
               auto_learn_housecode_.c_str(), static_cast<unsigned>(auto_learn_count_),
               static_cast<unsigned>(AUTO_LEARN_FRAMES_REQUIRED));
    }
    return;
  }

  // Phase 3: decode the frame against our housecode.
  const auto frame = decode_p109_frame(packet, housecode_);
  if (!frame.housecode_match) {
    return;
  }

  // Update debug frame-info sensor.
  last_rx_info_ = frame.info;
  if (last_rx_info_sensor_ != nullptr) last_rx_info_sensor_->publish_state(last_rx_info_);

  // Track last command per channel for status sensors — before PositionSource
  // filter so STOP, SAVE, motor-feedback all appear in the channel status.
  update_last_cmd_(frame, rssi);
  notify_covers_status_(frame.channel, frame.applies_to_all_channels);

  if (frame.position_source == PositionSource::None) {
    ESP_LOGD(TAG_HUB, "RX frame ignored for motor tracking: %s", frame.info.c_str());
    return;
  }

  // Self-TX echo guard.
  if (frame.position_source == PositionSource::RemoteInferred && last_self_tx_ms_ != 0 &&
      (millis() - last_self_tx_ms_) < SELF_TX_ECHO_GUARD_MS) {
    ESP_LOGD(TAG_HUB, "RX remote hint suppressed (self-TX echo within %u ms): %s",
             static_cast<unsigned>(SELF_TX_ECHO_GUARD_MS), frame.info.c_str());
    return;
  }

  // Apply motor_pct update.
  if (frame.applies_to_all_channels) {
    for (auto &v : motor_pct_) {
      v = frame.pct;
    }
  } else if (frame.channel < motor_pct_.size()) {
    motor_pct_[frame.channel] = frame.pct;
  }

  // Authoritative position info cancels any in-flight animation.
  if (frame.applies_to_all_channels) {
    for (uint32_t ch = 0; ch < interp_start_ms_.size(); ch++) {
      cancel_position_interpolation_(ch);
    }
  } else {
    cancel_position_interpolation_(frame.channel);
  }

  // A foreign remote frame supersedes any pending mid-position STOP and
  // invalidates in-flight calibration captures.
  if (frame.position_source == PositionSource::RemoteInferred) {
    cancel_mid_position_timers_(frame.channel, frame.applies_to_all_channels);
    if (frame.applies_to_all_channels) {
      for (uint32_t ch = 0; ch < cal_pending_tx_ms_.size(); ch++) {
        cancel_calibration_capture_(ch);
      }
    } else {
      cancel_calibration_capture_(frame.channel);
    }
  }

  // A motor-feedback frame may complete a pending calibration capture.
  bool cal_updated = false;
  if (frame.position_source == PositionSource::MotorFeedback) {
    cal_updated = try_resolve_calibration_(frame.channel, frame.pct);
    if (cal_updated) {
      persist_calibration_();
    }
  }

  const char *src = (frame.position_source == PositionSource::MotorFeedback) ? "motor update"
                                                                               : "remote hint";
  if (frame.applies_to_all_channels) {
    ESP_LOGI(TAG_HUB, "RX %s: channel=0 (all) pct=%d raw=%s", src, frame.pct,
             frame.raw.c_str());
  } else {
    ESP_LOGI(TAG_HUB, "RX %s: channel=%u pct=%d raw=%s", src, frame.channel, frame.pct,
             frame.raw.c_str());
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Public accessors
// ─────────────────────────────────────────────────────────────────────────────

int RojaflexHub::get_motor_pct(uint8_t channel) const {
  if (channel >= motor_pct_.size()) return -1;
  return motor_pct_[channel];
}

int RojaflexHub::get_cal_time_open_s(uint8_t channel) const {
  if (channel >= cal_time_open_s_.size()) return -1;
  return cal_time_open_s_[channel];
}

int RojaflexHub::get_cal_time_close_s(uint8_t channel) const {
  if (channel >= cal_time_close_s_.size()) return -1;
  return cal_time_close_s_[channel];
}

// ─────────────────────────────────────────────────────────────────────────────
// Actions called by covers
// ─────────────────────────────────────────────────────────────────────────────

void RojaflexHub::send_command(uint32_t channel_id, uint8_t cmd_code) {
  send_command_(channel_id, cmd_code, /*optimistic_update=*/true);
}

void RojaflexHub::send_command_(uint32_t channel_id, uint8_t cmd_code, bool optimistic_update) {
  if (channel_id >= MAX_CHANNELS) {
    ESP_LOGE(TAG_HUB, "Invalid channel_id: %u (allowed: 0..%u)",
             static_cast<unsigned>(channel_id), MAX_CHANNELS - 1);
    return;
  }
  if (!is_housecode_configured(housecode_)) {
    ESP_LOGW(TAG_HUB, "Shared housecode not configured");
    return;
  }

  const uint8_t channel = static_cast<uint8_t>(channel_id);
  const uint8_t up_code = static_cast<uint8_t>(Command::UP);
  const uint8_t down_code = static_cast<uint8_t>(Command::DOWN);

  std::vector<uint8_t> tx_packet;
  std::string final_msg;
  if (!build_tx_packet(housecode_, channel, cmd_code, tx_packet, final_msg)) {
    ESP_LOGE(TAG_HUB, "Failed to build TX packet");
    return;
  }
  ESP_LOGI(TAG_HUB, "Sending P109 packet: %s (x%u)", final_msg.c_str(),
           static_cast<unsigned>(tx_repetitions_));

  bool any_tx_ok = false;
  esphome::cc1101::CC1101Error last_err = esphome::cc1101::CC1101Error::NONE;
  for (uint8_t r = 0; r < tx_repetitions_; r++) {
    if (r > 0) delay(80);
    const auto err = transmit_(tx_packet);
    if (err == esphome::cc1101::CC1101Error::NONE) {
      any_tx_ok = true;
    } else {
      last_err = err;
    }
  }

  // Publish "ok" when at least one repeat succeeded, otherwise surface the
  // last attempt's error in short, technical form. Names mirror the
  // CC1101Error enum so log warnings and sensor state map 1:1.
  if (last_tx_sensor_ != nullptr) {
    const char *state = "ok";
    if (!any_tx_ok) {
      switch (last_err) {
        case esphome::cc1101::CC1101Error::TIMEOUT:  state = "timeout"; break;
        case esphome::cc1101::CC1101Error::PARAMS:   state = "params"; break;
        case esphome::cc1101::CC1101Error::PLL_LOCK: state = "pll-lock"; break;
        default: state = "fehler"; break;  // unreachable for transmit_packet today
      }
    }
    last_tx_sensor_->publish_state(state);
  }

  if (!any_tx_ok) {
    ESP_LOGW(TAG_HUB, "All %u TX repeats failed for channel=%u (last err=%d)",
             static_cast<unsigned>(tx_repetitions_), static_cast<unsigned>(channel),
             static_cast<int>(last_err));
    return;
  }

  // Any new command supersedes a running animation for this channel.
  cancel_position_interpolation_(channel_id);

  // Calibration capture decision.
  const int prev_pct = (channel < motor_pct_.size()) ? motor_pct_[channel] : -1;
  if (optimistic_update && cmd_code == up_code && prev_pct == 100) {
    arm_calibration_capture_(channel_id, 0);
  } else if (optimistic_update && cmd_code == down_code && prev_pct == 0) {
    arm_calibration_capture_(channel_id, 100);
  } else {
    cancel_calibration_capture_(channel_id);
  }

  if (!optimistic_update) return;

  int new_pct = -1;
  int direction_time_s = -1;
  if (cmd_code == up_code) {
    new_pct = 0;
    direction_time_s = get_cal_time_open_s(channel);
  } else if (cmd_code == down_code) {
    new_pct = 100;
    direction_time_s = get_cal_time_close_s(channel);
  }
  if (new_pct < 0 || channel >= motor_pct_.size()) return;  // STOP / unknown channel

  const bool can_interpolate =
      (direction_time_s > 0) && (prev_pct >= 0) && (prev_pct != new_pct);
  if (can_interpolate) {
    const int delta_pct = (new_pct > prev_pct) ? (new_pct - prev_pct) : (prev_pct - new_pct);
    const uint32_t duration_ms = (static_cast<uint32_t>(delta_pct) *
                                  static_cast<uint32_t>(direction_time_s) * 1000u) /
                                 100u;
    arm_position_interpolation_(channel_id, prev_pct, new_pct, duration_ms);
  } else {
    motor_pct_[channel] = new_pct;
  }
}

void RojaflexHub::set_position(uint32_t channel_id, int target_pct) {
  if (channel_id >= MAX_CHANNELS) {
    ESP_LOGE(TAG_HUB, "set_position: invalid channel_id=%u (allowed: 0..%u)",
             static_cast<unsigned>(channel_id), MAX_CHANNELS - 1);
    return;
  }

  if (channel_id == 0 && target_pct != 0 && target_pct != 100) {
    ESP_LOGW(TAG_HUB,
             "set_position channel=0 target=%d rejected: broadcast channel "
             "supports only end stops (0 or 100)",
             target_pct);
    return;
  }

  const int current_pct =
      (channel_id < motor_pct_.size()) ? motor_pct_[channel_id] : -1;
  const auto plan = compute_shutter_motion_plan(current_pct, target_pct,
                                                get_cal_time_open_s(channel_id),
                                                get_cal_time_close_s(channel_id));

  const std::string timer_name = mid_position_timer_name_(channel_id);
  esphome::App.scheduler.cancel_timeout(this, timer_name.c_str());

  const uint8_t up_code = static_cast<uint8_t>(Command::UP);
  const uint8_t down_code = static_cast<uint8_t>(Command::DOWN);
  const uint8_t stop_code = static_cast<uint8_t>(Command::STOP);

  using Action = ShutterMotionPlan::Action;

  switch (plan.action) {
    case Action::None:
      ESP_LOGW(TAG_HUB, "set_position channel=%u target=%d -> no action (%s)",
               static_cast<unsigned>(channel_id), plan.target_pct, plan.info);
      return;

    case Action::Stop:
      ESP_LOGI(TAG_HUB, "set_position channel=%u already at target=%d -> STOP",
               static_cast<unsigned>(channel_id), plan.target_pct);
      send_command_(channel_id, stop_code, true);
      return;

    case Action::UpToEnd:
    case Action::DownToEnd: {
      const bool going_up = (plan.action == Action::UpToEnd);
      ESP_LOGI(TAG_HUB, "set_position channel=%u target=%d -> %s (full %s)",
               static_cast<unsigned>(channel_id), plan.target_pct,
               going_up ? "UP" : "DOWN", going_up ? "open" : "close");
      send_command_(channel_id, going_up ? up_code : down_code, true);
      return;
    }

    case Action::UpThenStop:
    case Action::DownThenStop: {
      const bool going_up = (plan.action == Action::UpThenStop);
      ESP_LOGI(TAG_HUB, "set_position channel=%u from=%d target=%d -> %s for %u ms",
               static_cast<unsigned>(channel_id), current_pct, plan.target_pct,
               going_up ? "UP" : "DOWN", static_cast<unsigned>(plan.duration_ms));

      // Optimistic update disabled: motor_pct animates gradually, not jump.
      send_command_(channel_id, going_up ? up_code : down_code, false);
      arm_position_interpolation_(channel_id, current_pct, plan.target_pct, plan.duration_ms);

      const int captured_target = plan.target_pct;
      esphome::App.scheduler.set_timeout(
          this, timer_name.c_str(), plan.duration_ms,
          [this, channel_id, captured_target, stop_code]() {
            ESP_LOGI(TAG_HUB, "set_position timeout fired for channel=%u -> STOP",
                     static_cast<unsigned>(channel_id));
            send_command_(channel_id, stop_code, false);
            cancel_position_interpolation_(channel_id);
            if (channel_id < motor_pct_.size()) {
              motor_pct_[channel_id] = captured_target;
            }
          });
      return;
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// API actions
// ─────────────────────────────────────────────────────────────────────────────

bool RojaflexHub::apply_manual_housecode(const std::string &new_housecode) {
  if (!rojaflex::apply_manual_housecode(new_housecode, housecode_, auto_learn_housecode_,
                                        auto_learn_count_)) {
    return false;
  }
  ESP_LOGI(TAG_HUB, "Shared housecode set to: %s", housecode_.c_str());
  persist_housecode_();
  return true;
}

void RojaflexHub::note_ha_stop(uint8_t channel) {
  if (channel >= ha_stop_ring_.size()) return;
  auto &ring = ha_stop_ring_[channel];
  const uint32_t now = millis();

  // Shift left, append `now`. millis() == 0 is the empty-slot sentinel — for
  // 1 ms after boot a real timestamp collides with the sentinel, which only
  // breaks the very first window (gesture won't fire that early; harmless).
  for (size_t i = 0; i + 1 < ring.size(); i++) ring[i] = ring[i + 1];
  ring.back() = now;

  // Need a fully populated window inside the 2 s budget.
  if (ring.front() == 0) return;
  if (now - ring.front() > HA_STOP_RESET_WINDOW_MS) return;

  // Don't fire on channels that have nothing to reset — also avoids loud
  // warnings when the user is just rage-tapping STOP on an uncalibrated cover.
  const bool had_open = channel < cal_time_open_s_.size() && cal_time_open_s_[channel] >= 0;
  const bool had_close = channel < cal_time_close_s_.size() && cal_time_close_s_[channel] >= 0;
  if (!had_open && !had_close) return;

  ESP_LOGW(TAG_HUB,
           "%u STOPs within %u ms on channel=%u -> wiping calibration for this channel",
           static_cast<unsigned>(HA_STOP_RESET_PRESSES),
           static_cast<unsigned>(HA_STOP_RESET_WINDOW_MS), static_cast<unsigned>(channel));

  if (channel < cal_time_open_s_.size()) cal_time_open_s_[channel] = -1;
  if (channel < cal_time_close_s_.size()) cal_time_close_s_[channel] = -1;
  cancel_calibration_capture_(channel);
  persist_calibration_();

  // Wipe ring so the user can't double-fire by holding STOP one more time.
  ring = {};
}

// ─────────────────────────────────────────────────────────────────────────────
// Cover registration
// ─────────────────────────────────────────────────────────────────────────────

void RojaflexHub::register_cover(RojaflexCover *cover) {
  covers_.push_back(cover);
}

// ─────────────────────────────────────────────────────────────────────────────
// Status string helper (used by per-channel text sensor)
// ─────────────────────────────────────────────────────────────────────────────

std::string RojaflexHub::get_channel_status_string(uint8_t channel) const {
  const uint32_t ch = static_cast<uint32_t>(channel);
  if (ch >= motor_pct_.size()) return "config error";
  if (ch == 0) return "Broadcast channel - end stops only";

  const int pct = motor_pct_[ch];
  const int t_open = get_cal_time_open_s(channel);
  const int t_close = get_cal_time_close_s(channel);

  // Order matters: a freshly booted device with valid NVS calibration but no
  // motor frame yet should NOT report "Calibration needed" — that misleads
  // users into thinking the persisted times were lost. The real "needs
  // calibration" cases are checked first; position-resync is a separate state.
  const bool open_ok = (t_open >= 0);
  const bool close_ok = (t_close >= 0);

  if (!open_ok && !close_ok) return "Calibration needed: drive both end stops once";
  if (!open_ok) return "Calibration needed: drive fully open once";
  if (!close_ok) return "Calibration needed: drive fully closed once";

  if (pct < 0) return "Resync: drive once";

  // Show last received command + RSSI; fall back to "Calibrated" until first RX.
  if (channel < last_cmd_name_.size() && !last_cmd_name_[channel].empty()) {
    char buf[48];
    snprintf(buf, sizeof(buf), "%s (%.0f dBm)", last_cmd_name_[channel].c_str(),
             last_cmd_rssi_[channel]);
    return std::string(buf);
  }
  return "Calibrated";
}

// ─────────────────────────────────────────────────────────────────────────────
// TX helper
// ─────────────────────────────────────────────────────────────────────────────

esphome::cc1101::CC1101Error RojaflexHub::transmit_(const std::vector<uint8_t> &packet) {
  if (cc1101_ == nullptr) return esphome::cc1101::CC1101Error::PARAMS;
  // Stamp every TX attempt so the echo guard covers partial sends.
  last_self_tx_ms_ = millis();
  const auto err = cc1101_->transmit_packet(packet);
  if (err != esphome::cc1101::CC1101Error::NONE) {
    ESP_LOGW(TAG_HUB, "transmit_packet failed (error %d)", static_cast<int>(err));
  }
  return err;
}

// ─────────────────────────────────────────────────────────────────────────────
// Calibration capture helpers
// ─────────────────────────────────────────────────────────────────────────────

void RojaflexHub::cancel_calibration_capture_(uint32_t ch) {
  if (ch < cal_pending_tx_ms_.size()) cal_pending_tx_ms_[ch] = 0;
  if (ch < cal_pending_target_pct_.size()) cal_pending_target_pct_[ch] = -1;
}

void RojaflexHub::arm_calibration_capture_(uint32_t ch, int target_pct) {
  if (ch == 0) return;
  if (target_pct == 0 && ch < cal_time_open_s_.size() && cal_time_open_s_[ch] >= 0) return;
  if (target_pct == 100 && ch < cal_time_close_s_.size() && cal_time_close_s_[ch] >= 0) return;
  ensure_capacity(cal_pending_tx_ms_, ch + 1, static_cast<uint32_t>(0));
  ensure_capacity(cal_pending_target_pct_, ch + 1, -1);
  cal_pending_tx_ms_[ch] = millis();
  cal_pending_target_pct_[ch] = target_pct;
}

bool RojaflexHub::try_resolve_calibration_(uint32_t ch, int reported_pct) {
  if (ch >= cal_pending_tx_ms_.size()) return false;
  const uint32_t tx_ms = cal_pending_tx_ms_[ch];
  if (tx_ms == 0) return false;

  const int target =
      (ch < cal_pending_target_pct_.size()) ? cal_pending_target_pct_[ch] : -1;
  if (target < 0) return false;

  const uint32_t elapsed = millis() - tx_ms;
  if (elapsed >= CALIBRATION_CAPTURE_WINDOW_MS) {
    cancel_calibration_capture_(ch);
    return false;
  }
  if (reported_pct != target) return false;

  const int delta_s = static_cast<int>(elapsed / 1000u);
  if (delta_s < CALIBRATION_MIN_TIME_S) {
    ESP_LOGW(TAG_HUB, "Calibration capture rejected for channel=%u: %ds < %d s sanity floor",
             static_cast<unsigned>(ch), delta_s, CALIBRATION_MIN_TIME_S);
    cancel_calibration_capture_(ch);
    return false;
  }

  if (target == 0) {
    ensure_capacity(cal_time_open_s_, ch + 1, -1);
    cal_time_open_s_[ch] = delta_s;
    ESP_LOGI(TAG_HUB, "Calibrated channel=%u open time = %d s", static_cast<unsigned>(ch),
             delta_s);
  } else {
    ensure_capacity(cal_time_close_s_, ch + 1, -1);
    cal_time_close_s_[ch] = delta_s;
    ESP_LOGI(TAG_HUB, "Calibrated channel=%u close time = %d s", static_cast<unsigned>(ch),
             delta_s);
  }
  cancel_calibration_capture_(ch);
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Position interpolation helpers
// ─────────────────────────────────────────────────────────────────────────────

void RojaflexHub::cancel_position_interpolation_(uint32_t ch) {
  if (ch < interp_start_ms_.size()) interp_start_ms_[ch] = 0;
}

void RojaflexHub::arm_position_interpolation_(uint32_t ch, int start_pct, int end_pct,
                                               uint32_t duration_ms) {
  if (ch == 0 || duration_ms == 0 || start_pct == end_pct || start_pct < 0 || end_pct < 0) {
    cancel_position_interpolation_(ch);
    return;
  }
  ensure_capacity(interp_start_ms_, ch + 1, static_cast<uint32_t>(0));
  ensure_capacity(interp_start_pct_, ch + 1, -1);
  ensure_capacity(interp_end_pct_, ch + 1, -1);
  ensure_capacity(interp_duration_ms_, ch + 1, static_cast<uint32_t>(0));
  interp_start_ms_[ch] = millis();
  interp_start_pct_[ch] = start_pct;
  interp_end_pct_[ch] = end_pct;
  interp_duration_ms_[ch] = duration_ms;
}

void RojaflexHub::tick_position_interpolation_() {
  const uint32_t now = millis();
  const size_t n = interp_start_ms_.size();
  for (uint32_t ch = 0; ch < n; ch++) {
    if (interp_start_ms_[ch] == 0) continue;
    if (ch >= motor_pct_.size()) continue;

    const uint32_t duration = interp_duration_ms_[ch];
    if (duration == 0) {
      cancel_position_interpolation_(ch);
      continue;
    }
    const int end_pct = interp_end_pct_[ch];
    const uint32_t elapsed = now - interp_start_ms_[ch];
    // Saturation: commit the exact end_pct and stop ticking. The previous
    // version `continue`d here on the assumption that a motor-feedback frame
    // would write the final value, but motors that don't emit DEV=5 (or
    // commands not preceded by a scheduled STOP) leave motor_pct at whatever
    // the last sub-duration tick landed on — integer truncation pins that one
    // step below end_pct, so the HA slider sat at 1 % / 99 % indefinitely.
    if (elapsed >= duration) {
      motor_pct_[ch] = end_pct;
      cancel_position_interpolation_(ch);
      continue;
    }

    const int start_pct = interp_start_pct_[ch];
    const int delta = end_pct - start_pct;
    const int step = static_cast<int>((static_cast<int64_t>(delta) *
                                       static_cast<int64_t>(elapsed)) /
                                      static_cast<int64_t>(duration));
    motor_pct_[ch] = start_pct + step;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Mid-position scheduler helpers
// ─────────────────────────────────────────────────────────────────────────────

std::string RojaflexHub::mid_position_timer_name_(uint32_t ch) const {
  char buf[32];
  snprintf(buf, sizeof(buf), "rojaflex_pct_stop_%u", static_cast<unsigned>(ch));
  return std::string(buf);
}

void RojaflexHub::cancel_mid_position_timers_(uint8_t channel, bool all_channels) {
  if (all_channels) {
    for (uint32_t ch = 0; ch < MAX_CHANNELS; ch++) {
      esphome::App.scheduler.cancel_timeout(this, mid_position_timer_name_(ch).c_str());
    }
    return;
  }
  esphome::App.scheduler.cancel_timeout(this, mid_position_timer_name_(channel).c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
// Calibration serialization
// ─────────────────────────────────────────────────────────────────────────────

std::string RojaflexHub::serialize_calibration_() const {
  std::string result;
  const size_t n = std::max(cal_time_open_s_.size(), cal_time_close_s_.size());
  for (size_t ch = 0; ch < n; ch++) {
    const int o = (ch < cal_time_open_s_.size()) ? cal_time_open_s_[ch] : -1;
    const int c = (ch < cal_time_close_s_.size()) ? cal_time_close_s_[ch] : -1;
    if (o < 0 && c < 0) continue;
    if (!result.empty()) result += ";";
    char buf[24];
    snprintf(buf, sizeof(buf), "%u:%d,%d", static_cast<unsigned>(ch), o, c);
    result += buf;
  }
  return result;
}

void RojaflexHub::deserialize_calibration_(const std::string &s) {
  if (s.empty()) return;
  size_t pos = 0;
  while (pos < s.size()) {
    const size_t sep = std::min(s.find(';', pos), s.size());
    unsigned ch;
    int o, c;
    if (sscanf(s.c_str() + pos, "%u:%d,%d", &ch, &o, &c) == 3 && ch < MAX_CHANNELS) {
      ensure_capacity(cal_time_open_s_, ch + 1, -1);
      ensure_capacity(cal_time_close_s_, ch + 1, -1);
      if (o >= 0) cal_time_open_s_[ch] = o;
      if (c >= 0) cal_time_close_s_[ch] = c;
    }
    pos = sep + 1;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// NVS persistence
// ─────────────────────────────────────────────────────────────────────────────

void RojaflexHub::load_housecode_() {
  housecode_pref_ = esphome::global_preferences->make_preference<HousecodeStorage>(
      esphome::fnv1_hash("rojaflex_housecode"), true);
  HousecodeStorage stored{};
  if (housecode_pref_.load(&stored) && stored.value[0] != '\0') {
    // Only override the config default if the NVS value is configured (not "0000000").
    std::string nvs_hc(stored.value);
    if (is_housecode_configured(nvs_hc)) {
      housecode_ = nvs_hc;
      ESP_LOGI(TAG_HUB, "Loaded housecode from NVS: %s", housecode_.c_str());
    }
  }
}

void RojaflexHub::persist_housecode_() {
  HousecodeStorage storage{};
  strncpy(storage.value, housecode_.c_str(), sizeof(storage.value) - 1);
  housecode_pref_.save(&storage);
}

void RojaflexHub::load_calibration_() {
  cal_state_pref_ = esphome::global_preferences->make_preference<CalStorage>(
      esphome::fnv1_hash("rojaflex_cal"), true);
  CalStorage stored{};
  if (cal_state_pref_.load(&stored) && stored.value[0] != '\0') {
    deserialize_calibration_(std::string(stored.value));
    ESP_LOGI(TAG_HUB, "Loaded calibration from NVS");
  }
}

void RojaflexHub::persist_calibration_() {
  const std::string s = serialize_calibration_();
  CalStorage storage{};
  strncpy(storage.value, s.c_str(), sizeof(storage.value) - 1);
  cal_state_pref_.save(&storage);
}

// ─────────────────────────────────────────────────────────────────────────────
// Sensor publishing
// ─────────────────────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────────────────────
// Per-channel last-command tracking
// ─────────────────────────────────────────────────────────────────────────────

static const char *cmd_label(uint8_t cmd) {
  switch (cmd) {
    case 0x0: return "Stop";
    case 0x1: return "Up";
    case 0x8: return "Down";
    case 0x9: return "Save";
    case 0xD: return "Goto";
    case 0xE: return "Request";
    default:  return nullptr;
  }
}

void RojaflexHub::update_last_cmd_(const P109Frame &frame, float rssi) {
  char name[24];
  if (frame.device_type == 0x5) {
    snprintf(name, sizeof(name), "Pos. %d%%", frame.pct);
  } else {
    const char *label = cmd_label(frame.cmd);
    if (label != nullptr) {
      snprintf(name, sizeof(name), "%s", label);
    } else {
      snprintf(name, sizeof(name), "Cmd_%X", static_cast<unsigned>(frame.cmd));
    }
  }

  if (frame.applies_to_all_channels) {
    for (uint8_t ch = 0; ch < MAX_CHANNELS; ch++) set_last_cmd_(ch, name, rssi);
  } else {
    set_last_cmd_(frame.channel, name, rssi);
  }
}

void RojaflexHub::set_last_cmd_(uint8_t ch, const char *name, float rssi) {
  ensure_capacity(last_cmd_name_, ch + 1, std::string{});
  ensure_capacity(last_cmd_rssi_, ch + 1, 0.0f);
  last_cmd_name_[ch] = name;
  last_cmd_rssi_[ch] = rssi;
}

void RojaflexHub::notify_covers_status_(uint8_t ch, bool all_channels) {
  for (auto *cover : covers_) {
    if (all_channels || cover->get_channel() == ch) {
      cover->publish_status_now();
    }
  }
}

void RojaflexHub::publish_housecode_sensor_() {
  if (housecode_sensor_ == nullptr) return;

  if (is_housecode_configured(housecode_)) {
    housecode_sensor_->publish_state("0x" + housecode_);
    return;
  }

  if (!auto_learn_housecode_.empty()) {
    char buf[64];
    snprintf(buf, sizeof(buf), "Learning: 0x%s (%u/%u)",
             auto_learn_housecode_.c_str(), static_cast<unsigned>(auto_learn_count_),
             static_cast<unsigned>(AUTO_LEARN_FRAMES_REQUIRED));
    housecode_sensor_->publish_state(std::string(buf));
    return;
  }

  housecode_sensor_->publish_state("Receiving...");
}

}  // namespace rojaflex
