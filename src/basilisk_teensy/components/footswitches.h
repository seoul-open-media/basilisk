#pragma once

#include "../helpers/imports.h"
#include "../helpers/utils.h"

class FootSwitches {
 public:
  FootSwitches(int pin_l = 23, int pin_r = 29) : pins_{pin_l, pin_r} {}

  // Must be called before use.
  void Setup() {
    if (!setup_cplt_) {
      for (const auto& pin : pins_) pinMode(pin, INPUT);
      setup_cplt_ = true;
    }
  }

  // Should be called in regular interval to track feet contact to ground
  // and accumulation of such data.
  void Run() {
    if (!setup_cplt_) {
      Serial.println("FootSwitches: Setup NOT complete");
      return;
    }

    last_run_time = millis();

    for (uint8_t i = 0; i < 2; i++) {
      state_[i].contact >>= 1;
      if (digitalRead(pins_[i])) {
        state_[i].contact |= new_contact;
        state_[i].last_contact_time = millis();
      }
    }
  }

  const int pins_[2];
  struct State {
    uint64_t contact = 0;
    uint32_t last_contact_time = 0;

    bool Contact(uint8_t n) {
      return contact >= (((utils::one_uint64 << n) - 1) << (64 - n));
    }
  } state_[2];
  uint32_t last_run_time = 0;

 private:
  const uint64_t new_contact = utils::one_uint64 << 63;
  bool setup_cplt_ = false;
};
