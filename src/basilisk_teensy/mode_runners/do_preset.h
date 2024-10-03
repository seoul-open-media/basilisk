#pragma once

#include "../presets/matome.h"
#include "meta.h"

namespace preset {

namespace pivot {
double tgt_yaw;
PhiSpeed speed = 0.1;
}  // namespace pivot

namespace piv_spin {
PhiSpeed speed = 0.1;
}  // namespace piv_spin

}  // namespace preset

void ModeRunners::DoPreset(Basilisk* b) {
  auto& m = b->cmd_.mode;
  auto& c = b->cmd_.do_preset;

  switch (m) {
    case M::DoPreset: {
      if (1000 <= c.idx && c.idx <= 2999) {
        m = M::Pivot_Init;
        auto& p = b->cmd_.pivot;

        uint8_t digits[4];
        for (uint8_t i = 0; i < 4; i++) {
          digits[i] = c.idx % 10;
          c.idx /= 10;
        }

        p.didimbal = digits[3] == 1 ? BOOL_L : digits[3] == 2 ? BOOL_R : BOOL_L;
        preset::pivot::tgt_yaw = (digits[2] - 3) * 0.125;
        p.tgt_yaw = [](Basilisk* b) {
          return nearest_pmn(b->imu_.GetYaw(true), preset::pivot::tgt_yaw);
        };
        p.stride = 0.0;
        p.bend[IDX_L] = digits[1] == 1   ? 0.0
                        : digits[1] == 2 ? 0.125
                        : digits[1] == 3 ? -0.125
                        : digits[1] == 4 ? 0.25
                        : digits[1] == 5 ? -0.25
                                         : 0.0;
        p.bend[IDX_R] = digits[0] == 1   ? 0.0
                        : digits[0] == 2 ? -0.125
                        : digits[0] == 3 ? 0.125
                        : digits[0] == 4 ? -0.25
                        : digits[0] == 5 ? 0.25
                                         : 0.0;
        p.speed = preset::pivot::speed;
        p.acclim = 2.0;
        p.min_dur = 0;
        p.max_dur = 5000;
        p.exit_condition = nullptr;
        p.exit_to_mode = M::Idle_Init;

        return;
      }

      if (3100 <= c.idx && c.idx <= 3299) {
        m = M::PivSpin;
        auto& sp = b->cmd_.piv_spin;

        uint8_t digits[4];
        for (uint8_t i = 0; i < 4; i++) {
          digits[i] = c.idx % 10;
          c.idx /= 10;
        }

        sp.didimbal = digits[2] == 1   ? BOOL_L
                      : digits[2] == 2 ? BOOL_R
                                       : BOOL_L;
        sp.dest_yaw = digits[1] == 0 ? NaN
                                     : nearest_pmn(b->imu_.GetYaw(true),
                                                   (digits[1] - 3) * 0.125);
        sp.exit_thr = 0.01;
        sp.stride = digits[0] == 0 ? 0.125 : digits[0] / 36.0;
        if (!isnan(sp.dest_yaw)) {
          bool dest_is_greater = sp.dest_yaw > b->imu_.GetYaw(true);
          bool positive_stride_drives_greater = sp.didimbal == BOOL_L;
          if (dest_is_greater != positive_stride_drives_greater) {
            sp.stride *= -1.0;
          }
        }

        // Serial.print("cur_yaw ");
        // Serial.print(b->imu_.GetYaw(true), 4);
        // Serial.print("sp.dest_yaw ");
        // Serial.print(sp.dest_yaw, 4);
        // while (1);

        for (uint8_t f : IDX_LR) sp.bend[f] = 0.0;
        sp.speed = preset::piv_spin::speed;
        sp.acclim = 2.0;
        sp.min_stepdur = 0;
        sp.max_stepdur = -1;
        sp.interval = 0;
        sp.steps = -1;

        return;
      }

      auto* maybe_preset = SafeAt(Presets::presets, c.idx);
      if (maybe_preset) {
        (*maybe_preset)(b);
      } else {
        Serial.println("Unregistered preset index");
        m = M::Idle_Init;
      }
    } break;
    default:
      break;
  }
}
