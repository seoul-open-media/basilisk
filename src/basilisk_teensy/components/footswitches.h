#pragma once

#include "../helpers/imports.h"

class FootSwitches {
 public:
  FootSwitches(int pin_l = 23, int pin_r = 29)
      : pins_{pin_l, pin_r}, accums_{0} {}

  void Setup() {
    for (const auto& pin : pins_) pinMode(pin, INPUT);
  }

  uint8_t Poll() {
    uint8_t readings = 0;
    for (uint8_t i = 0; i < 2; i++) {
      const auto contact = digitalRead(pins_[i]);
      if (contact) Increment(accums_[i]);
      readings |= (1 << i);
    }
    return readings;
  }

  void ResetAccum() {
    for (auto& accum : accums_) accum = 0;
  }

  void Increment(uint32_t& val) {
    if (val + 1 != 0) val++;
  }

  const int pins_[2];
  uint32_t accums_[2];

 private:
  bool setup_cplt_ = false;
};
