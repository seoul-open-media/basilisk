#pragma once

#include <Arduino.h>

class Beat {
 public:
  Beat(const uint32_t& interval) : prev_beat_{millis()}, interval_{interval} {}

  bool Hit() {
    if (millis() - prev_beat_ >= interval_) {
      prev_beat_ += interval_;
      return true;
    } else {
      return false;
    }
  }

 private:
  uint32_t prev_beat_;
  const uint32_t interval_;
};
