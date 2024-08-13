#pragma once

#include <Arduino.h>

enum class EmStrength : int {
  Max = 0,
  Strong = 63,
  Medium = 127,
  Weak = 191,
  Min = 255,
};

class Ems {
 public:
  Ems(const uint8_t& pin_la = 3, const uint8_t& pin_lt = 4,
      const uint8_t& pin_ra = 5, const uint8_t& pin_rt = 6)
      : pin_la_{pin_la},
        pin_lt_{pin_lt},
        pin_ra_{pin_ra},
        pin_rt_{pin_rt},
        pins_{pin_la, pin_lt, pin_ra, pin_rt} {}

  // Part  | LeftAnkle | LeftToe | RightAnkle | RightToe
  // ID    | 1         | 2       | 3          | 4
  // Index | 0         | 1       | 2          | 3
  // Pin   | 3         | 4       | 5          | 6
  const uint8_t pins_[4], pin_la_, pin_lt_, pin_ra_, pin_rt_;

  void SetPinMode() {
    for (const auto& pin : pins_) pinMode(pin, OUTPUT);
  }

  void SetStrength(const uint8_t& id, const EmStrength& strength) {
    analogWrite(pins_[id - 1], (int)strength);
  }

  void FixAll() {
    for (auto pin : pins_) {
      SetStrength(pin, EmStrength::Max);
    }
  }

  void FreeAll() {
    for (auto pin : pins_) {
      SetStrength(pin, EmStrength::Min);
    }
  }

  void FixLeft() {
    SetStrength(1, EmStrength::Max);
    SetStrength(2, EmStrength::Max);
  }

  void FreeLeft() {
    SetStrength(1, EmStrength::Min);
    SetStrength(2, EmStrength::Min);
  }

  void FixRight() {
    SetStrength(3, EmStrength::Max);
    SetStrength(4, EmStrength::Max);
  }

  void FreeRight() {
    SetStrength(3, EmStrength::Min);
    SetStrength(4, EmStrength::Min);
  }

  void FixAnkles() {
    SetStrength(1, EmStrength::Max);
    SetStrength(3, EmStrength::Max);
  }

  void FreeAnkles() {
    SetStrength(1, EmStrength::Min);
    SetStrength(3, EmStrength::Min);
  }

  void FixToes() {
    SetStrength(2, EmStrength::Max);
    SetStrength(4, EmStrength::Max);
  }

  void FreeToes() {
    SetStrength(2, EmStrength::Min);
    SetStrength(4, EmStrength::Min);
  }
};
