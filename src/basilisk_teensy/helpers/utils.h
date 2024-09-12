#pragma once

#include "imports.h"

namespace utils {

// Time is in milliseconds, stored as uint32_t.
// Continuously usable up to approximately 50 days.
class Beat {
 public:
  Beat(const uint32_t& interval)
      : next_beat_{millis() + interval}, interval_{interval} {}

  bool Hit() {
    if (millis() >= next_beat_) {
      while (millis() >= next_beat_) next_beat_ += interval_;
      return true;
    } else {
      return false;
    }
  }

 private:
  uint32_t next_beat_;
  const uint32_t interval_;
};

}  // namespace utils
