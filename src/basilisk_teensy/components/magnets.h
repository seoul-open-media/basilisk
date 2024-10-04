#pragma once

#include "../helpers/imports.h"
#include "lego_blocks.h"

enum class MagStren : uint8_t {
  Max = 0,
  Strong = 63,
  Medium = 127,
  Weak = 191,
  Min = 255
};

MagStren Bool2MS(const bool& attach) {
  return attach ? MagStren::Max : MagStren::Min;
}

// Part | LeftAnkle | LeftToe | RightAnkle | RightToe
// ID   | 0         | 1       | 2          | 3
// Pin  | 3         | 4       | 5          | 6

class Magnets {
 public:
  Magnets(LegoBlocks* lego,  //
          const uint8_t& pin_la = 3, const uint8_t& pin_lt = 4,
          const uint8_t& pin_ra = 5, const uint8_t& pin_rt = 6)
      : pins_{pin_la, pin_lt, pin_ra, pin_rt}, lego_{lego} {}

  // Must be called before use.
  bool Setup() {
    for (const auto& pin : pins_) pinMode(pin, OUTPUT);
    AttachAll();

    if (!lego_) {
      Serial.println("Magnets: Failed to set reference to Lego component");
      return false;
    }

    Serial.println("Magnets: Setup complete");
    return true;
  }

  // Should be called in regular interval to track if any of the
  // electromagnets are being passed current for over 3 seconds.
  void Run() {
    for (uint8_t id = 0; id < 4; id++) {
      if (attaching_[id]) {
        last_attach_time_[id] = millis();
      } else {
        last_release_time_[id] = millis();
      }
      time_since_last_attach_[id] = millis() - last_attach_time_[id];
      heavenfall_warning_[id] = (time_since_last_attach_[id] > 5000);
    }
  }

  void SetStrength(const uint8_t& id, const MagStren& strength) {
    analogWrite(pins_[id], static_cast<int>(strength));
    attaching_[id] = (strength == MagStren::Max);
    lego_->Reset();
  }

  void AttachAll() {
    for (uint8_t id = 0; id < 4; id++) {
      SetStrength(id, MagStren::Max);
    }
  }

  void ReleaseAll() {
    for (uint8_t id = 0; id < 4; id++) {
      SetStrength(id, MagStren::Min);
    }
  }

  const uint8_t pins_[4];
  bool attaching_[4] = {false, false, false, false};
  uint32_t last_attach_time_[4] = {0, 0, 0, 0};
  uint32_t last_release_time_[4] = {0, 0, 0, 0};
  uint32_t time_since_last_attach_[4] = {0, 0, 0, 0};
  bool heavenfall_warning_[4] = {false, false, false, false};
  LegoBlocks* lego_;
};
