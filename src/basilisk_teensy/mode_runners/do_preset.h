#pragma once

#include "../presets/matome.h"
#include "meta.h"

namespace preset {

namespace pivot {
double tgt_yaw;
PhiSpeed speed = globals::stdval::speed::normal;
}  // namespace pivot

namespace piv_spin {
PhiSpeed speed = globals::stdval::speed::normal;
}  // namespace piv_spin

}  // namespace preset

void ModeRunners::DoPreset(Basilisk* b) {
  auto& m = b->cmd_.mode;
  auto& c = b->cmd_.do_preset;
  auto idx = c.idx;  // Copy, not reference.

  switch (m) {
    case M::DoPreset: {
      if (1000 <= idx && idx <= 2999) {  // PPP Pivot range
        uint8_t digits[4];
        for (uint8_t i = 0; i < 4; i++) {
          digits[i] = idx % 10;
          idx /= 10;
        }

        m = M::Pivot_Init;
        auto& p = b->cmd_.pivot;
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
        p.acclim = globals::stdval::acclim::normal;
        p.min_dur = 0;
        p.max_dur = 3000;
        p.exit_condition = nullptr;
        p.exit_to_mode = M::Idle_Init;

        return;
      }

      if (3100 <= idx && idx <= 3299) {  // PPP PivSpin range
        uint8_t digits[3];
        for (uint8_t i = 0; i < 3; i++) {
          digits[i] = idx % 10;
          idx /= 10;
        }

        m = M::PivSpin;
        auto& p = b->cmd_.piv_spin;
        p.didimbal = digits[2] == 1 ? BOOL_L : digits[2] == 2 ? BOOL_R : BOOL_L;
        p.dest_yaw = digits[1] == 0 ? NaN
                                    : nearest_pmn(b->imu_.GetYaw(true),
                                                  (digits[1] - 3) * 0.125);
        p.exit_thr = 0.01;
        p.stride = digits[0] == 0 ? 0.125 : digits[0] / 36.0;
        if (!isnan(p.dest_yaw)) {
          bool dest_is_greater = p.dest_yaw > b->imu_.GetYaw(true);
          bool positive_stride_drives_greater = p.didimbal == BOOL_L;
          if (dest_is_greater != positive_stride_drives_greater) {
            p.stride *= -1.0;
          }
        }
        for (uint8_t f : IDX_LR) p.bend[f] = 0.0;
        p.speed = preset::piv_spin::speed;
        p.acclim = 2.0;
        p.min_stepdur = 0;
        p.max_stepdur = -1;
        p.interval = 0;
        p.steps = -1;

        return;
      }

      // TODO: PPP for Sufi, etc.

      auto* maybe_preset = SafeAt(Presets::presets, idx);
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
