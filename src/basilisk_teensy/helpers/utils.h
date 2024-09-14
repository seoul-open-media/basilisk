#pragma once

#include "imports.h"

namespace utils {

const uint64_t one_uint64 = static_cast<uint64_t>(1);

double signedpow(const double& base, const float& exponent) {
  const auto y = pow(abs(base), exponent);
  return base >= 0 ? y : -y;
}

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
