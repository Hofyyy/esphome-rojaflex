// rojaflex_cover.cpp
#include "rojaflex_cover.h"
#include "../rojaflex.h"
#include "../p109_protocol.h"

#include <cmath>

namespace rojaflex {

static const char *const TAG = "rojaflex.cover";

void RojaflexCover::setup() {
  // Position unknown until first motor feedback or remote frame.
  this->position = esphome::cover::COVER_OPEN;
}

void RojaflexCover::loop() {
  const uint32_t now = millis();

  // Update cover position every 200 ms from hub motor_pct.
  if (now - last_update_ms_ >= 200) {
    last_update_ms_ = now;

    const int pct = parent_->get_motor_pct(channel_);
    if (pct >= 0) {
      // Protocol: 0 = fully open, 100 = fully closed.
      // ESPHome: 1.0 = fully open, 0.0 = fully closed.
      const float new_pos = 1.0f - static_cast<float>(pct) / 100.0f;
      if (fabsf(new_pos - this->position) > 0.005f) {
        this->position = new_pos;
        this->publish_state();
      }
    }
  }

  // Update per-channel calibration status sensor every 5 s.
  if (status_sensor_ != nullptr && now - last_status_update_ms_ >= 5000) {
    last_status_update_ms_ = now;
    status_sensor_->publish_state(parent_->get_channel_status_string(channel_));
  }
}

void RojaflexCover::publish_status_now() {
  if (status_sensor_ != nullptr) {
    status_sensor_->publish_state(parent_->get_channel_status_string(channel_));
  }
}

void RojaflexCover::dump_config() {
  LOG_COVER("", "Rojaflex Cover", this);
  ESP_LOGCONFIG(TAG, "  Channel: %u", static_cast<unsigned>(channel_));
}

float RojaflexCover::get_setup_priority() const {
  return esphome::setup_priority::DATA - 1.0f;
}

esphome::cover::CoverTraits RojaflexCover::get_traits() {
  auto traits = esphome::cover::CoverTraits();
  traits.set_supports_position(true);
  traits.set_supports_stop(true);
  traits.set_is_assumed_state(false);
  return traits;
}

void RojaflexCover::control(const esphome::cover::CoverCall &call) {
  if (call.get_stop()) {
    parent_->note_ha_stop(channel_);
    parent_->send_command(channel_, static_cast<uint8_t>(Command::STOP));
    return;
  }
  if (call.get_position().has_value()) {
    const float pos = *call.get_position();
    // ESPHome convention: 1.0 = open, 0.0 = closed.
    // Use send_command for the end stops (full OPEN / CLOSE) so the
    // calibration capture state machine sees them as plain UP/DOWN
    // commands, not as software-timed mid-position drives.
    if (pos >= esphome::cover::COVER_OPEN - 0.001f) {
      parent_->send_command(channel_, static_cast<uint8_t>(Command::UP));
    } else if (pos <= esphome::cover::COVER_CLOSED + 0.001f) {
      parent_->send_command(channel_, static_cast<uint8_t>(Command::DOWN));
    } else {
      // Protocol percentage: 0 = fully open, 100 = fully closed.
      const int target_pct = static_cast<int>((1.0f - pos) * 100.0f + 0.5f);
      parent_->set_position(channel_, target_pct);
    }
  }
}

}  // namespace rojaflex
