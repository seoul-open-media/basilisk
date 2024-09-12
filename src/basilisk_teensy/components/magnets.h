#pragma once

#include "../helpers/imports.h"

enum class MagnetStrength : uint8_t {
  Max = 0,
  Strong = 63,
  Medium = 127,
  Weak = 191,
  Min = 255,
};

// Part | LeftAnkle | LeftToe | RightAnkle | RightToe
// ID   | 0         | 1       | 2          | 3
// Pin  | 3         | 4       | 5          | 6

class Magnets {
 public:
  Magnets(const uint8_t& pin_la = 3, const uint8_t& pin_lt = 4,
          const uint8_t& pin_ra = 5, const uint8_t& pin_rt = 6)
      : pins_{pin_la, pin_lt, pin_ra, pin_rt} {}

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
    for (uint8_t i = 0; i < 4; i++) {
      heavenfall_warning_[i] = (millis() - last_fix_time_[i] > 3000);
    }
  }

  uint32_t TimeSinceLastFix(const uint8_t& id) {
    if (id < 4) {
      return millis() - last_fix_time_[id];
    } else {
      return -1;
    }
  }

  void SetStrength(const uint8_t& id, const MagnetStrength& strength) {
    analogWrite(pins_[id], static_cast<int>(strength));
    if (strength == MagnetStrength::Max) last_fix_time_[id] = millis();
  }

  void FixAll() {
    for (auto pin : pins_) {
      SetStrength(pin, MagnetStrength::Max);
    }
  }

  void FreeAll() {
    for (auto pin : pins_) {
      SetStrength(pin, MagnetStrength::Min);
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
  uint32_t last_fix_time_[4] = {0, 0, 0, 0};
  bool heavenfall_warning_[4] = {false, false, false, false};
};
