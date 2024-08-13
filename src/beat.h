#pragma once

#include <Arduino.h>

// Usable up to approx. 50 days.
class Beat {
 public:
  Beat(const uint32_t& interval_ms)
      : next_beat_{millis() + interval_ms}, interval_{interval_ms} {}

  bool Hit() {
    if (millis() >= next_beat_) {
      while (next_beat_ < millis()) next_beat_ += interval_;
      return true;
    } else {
      return false;
    }
  }

 private:
  uint32_t next_beat_;
  const uint32_t interval_;
};
