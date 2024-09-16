#pragma once

#include "../helpers/imports.h"
#include "lego_blocks.h"

enum class MagnetStrength : uint8_t {
  Max = 0,
  Strong = 63,
  Medium = 127,
  Weak = 191,
  Min = 255,
};

MagnetStrength Bool2MS(bool fix) {
  return fix ? MagnetStrength::Max : MagnetStrength::Min;
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
    FixAll();
    Serial.println("Magnets: Setup complete");
    return true;
  }

  // Should be called in regular interval to track if any of the
  // electromagnets are being passed current for over 3 seconds.
  void Run() {
    for (uint8_t id = 0; id < 4; id++) {
      if (fixing_[id]) last_fix_time_[id] = millis();
      time_since_last_fix_[id] = millis() - last_fix_time_[id];
      heavenfall_warning_[id] = (time_since_last_fix_[id] > 3000);
    }
  }

  void SetStrength(const uint8_t& id, const MagnetStrength& strength) {
    analogWrite(pins_[id], static_cast<int>(strength));
    fixing_[id] = (strength == MagnetStrength::Max);
    lego_->Reset();
  }

  void FixAll() {
    for (uint8_t id = 0; id < 4; id++) {
      SetStrength(id, MagnetStrength::Max);
    }
  }

  void FreeAll() {
    for (uint8_t id = 0; id < 4; id++) {
      SetStrength(id, MagnetStrength::Min);
    }
  }

  void FixLeft() {
    SetStrength(0, MagnetStrength::Max);
    SetStrength(1, MagnetStrength::Max);
  }

  void FreeLeft() {
    SetStrength(0, MagnetStrength::Min);
    SetStrength(1, MagnetStrength::Min);
  }

  void FixRight() {
    SetStrength(2, MagnetStrength::Max);
    SetStrength(3, MagnetStrength::Max);
  }

  void FreeRight() {
    SetStrength(2, MagnetStrength::Min);
    SetStrength(3, MagnetStrength::Min);
  }

  void FixAnkles() {
    SetStrength(0, MagnetStrength::Max);
    SetStrength(2, MagnetStrength::Max);
  }

  void FreeAnkles() {
    SetStrength(0, MagnetStrength::Min);
    SetStrength(2, MagnetStrength::Min);
  }

  void FixToes() {
    SetStrength(1, MagnetStrength::Max);
    SetStrength(3, MagnetStrength::Max);
  }

  void FreeToes() {
    SetStrength(1, MagnetStrength::Min);
    SetStrength(3, MagnetStrength::Min);
  }

  const uint8_t pins_[4];
  bool fixing_[4] = {false, false, false, false};
  uint32_t last_fix_time_[4] = {0, 0, 0, 0};
  uint32_t time_since_last_fix_[4] = {0, 0, 0, 0};
  bool heavenfall_warning_[4] = {false, false, false, false};
  LegoBlocks* lego_;
};
