#pragma once

#include "../helpers/imports.h"

/* Length unit of incoming bytes from the LPS board is '10cm', so the field
 * `uin8_t dists_raw_[3]` which saves the raw values follows this unit, but the
 * rest of the program assumes 'cm' as length unit, for convenience in intuitive
 * choreography.
 *
 * Assumptions:
 * - Anchors ABC and the LPS board on Basilisk are on the same height.
 * - Anchor A = (0, 0); Anchor B = (c, 0); Anchor C = (x_c, y_c);
 *   with c > 0; y_c > 0.
 * - Basilisk never reaches beyond Anchor C in y axis, i.e. y < y_c.
 *
 * Trilateration:
 * (1):         x^2 + y^2         = a^2
 * (2):   (x - c)^2 + y^2         = b^2
 * (3): (x - x_c)^2 + (y - y_c)^2 = c^2
 *
 * (4) = (1) - (2): 2cx - c^2 = a^2 - b^2
 *                  x = (a^2 - b^2 + c^2) / (2c)
 * From (3): y = y_c +- sqrt(c^2 - (x - x_c)^2)
 *           is computable since we already have computed x at (4),
 *           and we take minus since we assumed y < y_c. */

#define LPS_SERIAL (Serial6)

class Lps {
 public:
  Lps(const double& c, const double& x_c, const double& y_c)
      : cfg_{.c = c, .x_c = x_c, .y_c = y_c} {}

  // Must be called before use.
  bool Setup() {
    LPS_SERIAL.begin(9600);
    if (!LPS_SERIAL) {
      Serial.println("LPS: LPS_SERIAL(Serial6) begin failed");
      return false;
    }
    for (auto& dist_sm : dists_sm_) dist_sm.begin(SMOOTHED_AVERAGE, 10);
    Serial.println("LPS: Setup complete");
    return true;
  }

  // Should be called continuously to immediately receive to
  // incoming sensor data and prevent Serial buffer overflow.
  void Run() {
    if (LPS_SERIAL.available() >= 6) {
      if (LPS_SERIAL.read() == 255 && LPS_SERIAL.read() == 2) {
        error_.matome = 0;
        for (auto i = 0; i < 3; i++) {
          const auto raw = LPS_SERIAL.read();
          if (raw < 250) {
            dists_raw_[i] = raw;
            dists_sm_[i].add(10.0 * raw);
          } else {
            error_.bytes[i] = raw;
          }
        }
        latency_ = LPS_SERIAL.read();
        if (!error_.matome) SetXY();
      } else {
        while (LPS_SERIAL.available()) LPS_SERIAL.read();
      }
      last_raw_update_ = millis();
    }
  }

 private:
  void SetXY() {
    const auto a = dists_sm_[0].get();
    const auto b = dists_sm_[1].get();
    const auto c = dists_sm_[2].get();

    x_ = (sq(a) - sq(b) + sq(cfg_.c)) / (2 * cfg_.c);
    y_ = cfg_.y_c - sqrt(sq(c) - sq(x_ - cfg_.x_c));

    last_xy_update_ = millis();
  }

 public:
  uint8_t dists_raw_[3] = {0, 0, 0};
  union {
    uint8_t bytes[3];
    uint32_t matome = 0;
  } error_;
  uint8_t latency_ = 0;
  uint32_t last_raw_update_ = 0;
  Smoothed<double> dists_sm_[3];
  double x_ = 0.0, y_ = 0.0;
  uint32_t last_xy_update_ = 0;
  const struct {
    const double c, x_c, y_c;
  } cfg_;
};
