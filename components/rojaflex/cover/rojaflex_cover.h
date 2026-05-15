// rojaflex_cover.h
// Per-channel cover entity. Delegates open/close/stop/set_position to the
// RojaflexHub and reports current position from hub->get_motor_pct().

#pragma once

#include "esphome/components/cover/cover.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/component.h"

namespace rojaflex {

class RojaflexHub;  // forward declaration

class RojaflexCover : public esphome::cover::Cover, public esphome::Component {
 public:
  // ── Configuration setters ───────────────────────────────────────────────

  void set_rojaflex_parent(RojaflexHub *hub) { parent_ = hub; }
  void set_channel(uint8_t channel) { channel_ = channel; }
  void set_status_sensor(esphome::text_sensor::TextSensor *s) { status_sensor_ = s; }

  uint8_t get_channel() const { return channel_; }
  // Called by hub when a new command arrives on this channel to update the
  // status sensor immediately rather than waiting for the 5 s poll.
  void publish_status_now();

  // ── Component lifecycle ─────────────────────────────────────────────────

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override;

  // ── Cover interface ─────────────────────────────────────────────────────

  esphome::cover::CoverTraits get_traits() override;
  void control(const esphome::cover::CoverCall &call) override;

 private:
  RojaflexHub *parent_{nullptr};
  uint8_t channel_{0};
  esphome::text_sensor::TextSensor *status_sensor_{nullptr};

  uint32_t last_update_ms_{0};
  uint32_t last_status_update_ms_{0};
};

}  // namespace rojaflex
