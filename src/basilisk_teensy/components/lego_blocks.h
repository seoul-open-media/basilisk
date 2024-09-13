#pragma once

#include "../helpers/imports.h"
#include "../helpers/utils.h"

/* Stores the history of feet contact to groud to two uint64_t's,
 * each for each foot in `state_[01].contact`. MSB contains recent data,
 * and LSB oldest, 1 representing contact and 0 detachment.
 * Check if contact has continued for n last poll by `state_.Contact(n)`.
 * Why named LegoBlocks? Cuz you scream when you step on them. */
class LegoBlocks {
 public:
  LegoBlocks(const int& pin_l = 23, const int& pin_r = 29)
      : pins_{pin_l, pin_r} {}

  // Must be called before use.
  bool Setup() {
    for (const auto& pin : pins_) pinMode(pin, INPUT);
    Serial.println("LegoBlocks: Setup complete");
    return true;
  }

  // Should be called in regular interval to track history of contact.
  void Run() {
    for (uint8_t i = 0; i < 2; i++) {
      state_[i].contact >>= 1;
      if (digitalRead(pins_[i])) {
        state_[i].contact |= new_contact;
        state_[i].last_contact_time = millis();
      }
    }

    last_run_time = millis();
  }

  const int pins_[2];
  struct {
    uint64_t contact = 0;
    uint32_t last_contact_time = 0;
    bool Contact(uint8_t n) {
      if (n == 0 || n > 64) {
        Serial.println("LegoBlocks::Contact: Bad argument");
        return false;
      }

      return !(~contact >> (64 - n));
    }
    bool Detached(uint8_t n) {
      if (n == 0 || n > 64) {
        Serial.println("LegoBlocks::Detached: Bad argument");
        return false;
      }

      return !(contact >> (64 - n));
    }
  } state_[2];
  uint32_t last_run_time = 0;

 private:
  const uint64_t new_contact = utils::one_uint64 << 63;
};
