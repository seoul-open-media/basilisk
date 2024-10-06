#pragma once

#include "../presets/matome.h"
#include "meta.h"

namespace do_preset {

namespace pivot {
double tgt_yaw;
PhiSpeed speed = globals::stdval::speed::normal;
}  // namespace pivot

namespace piv_spin {
PhiSpeed speed = globals::stdval::speed::normal;
}  // namespace piv_spin

}  // namespace do_preset

void ModeRunners::DoPreset(Basilisk* b) {
  auto& m = b->cmd_.mode;
  auto idx = b->cmd_.do_preset.idx;  // Copy, not reference.

  switch (m) {
    case M::DoPreset: {
      if (1000 <= idx && idx <= 2999) {  // PPP Pivot range
                                         // Decimal ABCD
                                         // A = didim = 1 ~ 2
                                         //       == 1 -> L
                                         //       == 2 -> R
                                         // B = tgt_yaw = 1 ~ 8
                                         //       == 1 ~ 8 -> S, SE, E, NE, ...
                                         // C = bend_l = 1 ~ 6
                                         //       == 1 -> Straight
                                         //       == 2 -> Palja
                                         //       == 3 -> Morasou
                                         //       == 4 -> Outward
                                         //       == 5 -> Awkward
                                         // D = bend_r
                                         //       == Same as C
        uint8_t digits[4];
        for (uint8_t i = 0; i < 4; i++) {
          digits[i] = idx % 10;
          idx /= 10;
        }

        m = M::Pivot_Init;
        auto& c = b->cmd_.pivot;
        c.didimbal = digits[3] == 1 ? BOOL_L : BOOL_R;
        do_preset::pivot::tgt_yaw = (digits[2] - 3) * 0.125;
        c.tgt_yaw = [](Basilisk* b) {
          return nearest_pmn(b->imu_.GetYaw(true), do_preset::pivot::tgt_yaw);
        };
        c.stride = 0.0;
        c.bend[IDX_L] = digits[1] == 1   ? 0.0
                        : digits[1] == 2 ? 0.125
                        : digits[1] == 3 ? -0.125
                        : digits[1] == 4 ? 0.25
                        : digits[1] == 5 ? -0.25
                                         : 0.0;
        c.bend[IDX_R] = digits[0] == 1   ? 0.0
                        : digits[0] == 2 ? -0.125
                        : digits[0] == 3 ? 0.125
                        : digits[0] == 4 ? -0.25
                        : digits[0] == 5 ? 0.25
                                         : 0.0;
        c.speed = do_preset::pivot::speed;
        c.acclim = globals::stdval::acclim::normal;
        c.min_dur = 0;
        c.max_dur = globals::stdval::maxdur::safe;
        c.exit_condition = nullptr;
        c.exit_to_mode = M::Idle_Init;

        return;
      }

      if (3100 <= idx && idx <= 3299) {  // PPP PivSpin range
                                         // Decimal 3ABC
                                         // A = didim = 1 ~ 2
                                         //       == 1 -> L
                                         //       == 2 -> R
                                         // B = dest_yaw = 0 ~ 8
                                         //       == 1 ~ 8 -> S, SE, E, NE, ...
                                         //       == 0     -> NaN
                                         // C = stride = 0 ~ 9
                                         //       == 1 ~ 9 -> * 10 deg
                                         //       == 0     -> 45 deg
        uint8_t digits[3];
        for (uint8_t i = 0; i < 3; i++) {
          digits[i] = idx % 10;
          idx /= 10;
        }

        m = M::PivSpin;
        auto& c = b->cmd_.piv_spin;
        c.didimbal = digits[2] == 1 ? BOOL_L : BOOL_R;
        c.dest_yaw = digits[1] == 0 ? NaN
                                    : nearest_pmn(b->imu_.GetYaw(true),
                                                  (digits[1] - 3) * 0.125);
        c.exit_thr = 0.01;
        c.stride = digits[0] == 0 ? 0.125 : digits[0] / 36.0;
        if (!isnan(c.dest_yaw)) {
          bool dest_is_greater = c.dest_yaw > b->imu_.GetYaw(true);
          bool positive_stride_drives_greater = c.didimbal == BOOL_L;
          if (dest_is_greater != positive_stride_drives_greater) {
            c.stride *= -1.0;
          }
        }
        for (uint8_t f : IDX_LR) c.bend[f] = 0.0;
        c.speed = do_preset::piv_spin::speed;
        c.acclim = globals::stdval::acclim::normal;
        c.min_stepdur = 0;
        c.max_stepdur = globals::stdval::maxdur::safe;
        c.interval = 0;
        c.steps = -1;

        return;
      }

      if (3300 <= idx && idx <= 3399) {  // PPP Sufi range
                                         // Decimal 33AB
                                         // A = dest_yaw = 0 ~ 8
                                         //       == 0     -> NaN
                                         //       == 1 ~ 8 -> S, SE, E, NE, ...
                                         // B = stride = 0 ~ 9
                                         //       == 0     -> 45 deg
                                         //       == 1 ~ 9 -> * 10 deg
        uint8_t digits[2];
        for (uint8_t i = 0; i < 2; i++) {
          digits[i] = idx % 10;
          idx /= 10;
        }

        m = M::Sufi;
        auto& c = b->cmd_.sufi;
        c.init_didimbal = BOOL_L;
        c.dest_yaw = digits[1] == 0 ? NaN
                                    : nearest_pmn(b->imu_.GetYaw(true),
                                                  (digits[1] - 3) * 0.125);
        c.exit_thr = 0.01;
        c.stride = digits[0] == 0 ? 0.125 : digits[0] / 36.0;
        if (!isnan(c.dest_yaw)) {
          bool dest_is_greater = c.dest_yaw > b->imu_.GetYaw(true);
          if (!dest_is_greater) {
            c.stride *= -1.0;
          }
        }
        for (uint8_t f : IDX_LR) c.bend[f] = 0.0;
        c.speed = do_preset::piv_spin::speed;
        c.acclim = globals::stdval::acclim::normal;
        c.min_stepdur = 0;
        c.max_stepdur = globals::stdval::maxdur::safe;
        c.interval = 0;
        c.steps = -1;

        return;
      }

      if (4000 <= idx && idx <= 4999) {  // PPP WalkToDir range
                                         // Decimal 4ABC
                                         // A = tgt_yaw = 0 ~ 8
                                         //       == 0     -> NaN
                                         //       == 1 ~ 8 -> NSEW
                                         // B = stride = 0 ~ 9
                                         //       == 0     -> 45 deg
                                         //       == 1 ~ 9 -> * 10 deg
                                         // C = steps
                                         //       == 0     -> 10
                                         //       == 1 ~ 9 -> =
        uint8_t digits[3];
        for (uint8_t i = 0; i < 3; i++) {
          digits[i] = idx % 10;
          idx /= 10;
        }

        m = M::WalkToDir;
        auto& c = b->cmd_.walk_to_dir;
        c.init_didimbal = BOOL_L;
        c.tgt_yaw = digits[2] == 0 ? NaN
                                   : nearest_pmn(b->imu_.GetYaw(true),
                                                 (digits[2] - 3) * 0.125);
        c.stride = digits[1] == 0 ? 0.125 : digits[1] / 36.0;
        if (abs(c.tgt_yaw - b->imu_.GetYaw(true)) > 0.25) {
          c.tgt_yaw = nearest_pmn(b->imu_.GetYaw(true), c.tgt_yaw + 0.5);
          c.stride *= -1.0;
        }
        for (uint8_t f : IDX_LR) c.bend[f] = 0.0;
        c.speed = globals::stdval::speed::normal;
        c.min_stepdur = 0;
        c.max_stepdur = globals::stdval::maxdur::safe;
        c.steps = digits[0] == 0 ? 10 : digits[0];

        return;
      }

      if (10000 <= idx && idx <= 19999) {  // PPP WalkToPos range
                                           // Decimal 1ABCD
                                           // (AB) = tgt_pos_x
                                           //          == (AB) * 10cm
                                           // (CD) = tgt_pos_y
                                           //          == (CD) * 10cm
        uint8_t digits[4];
        for (uint8_t i = 0; i < 4; i++) {
          digits[i] = idx % 10;
          idx /= 10;
        }

        m = M::WalkToPos;
        auto& c = b->cmd_.walk_to_pos;
        c.init_didimbal = BOOL_L;
        double x = (10 * digits[3] + digits[2]) * 10.0;
        double y = (10 * digits[1] + digits[0]) * 10.0;
        c.tgt_pos = Vec2{x, y};
        c.dist_thr = 25;
        c.stride = 0.125;
        for (uint8_t f : IDX_LR) c.bend[f] = 0.0;
        c.speed = globals::stdval::speed::normal;
        c.acclim = globals::stdval::acclim::normal;
        c.min_stepdur = 0;
        c.max_stepdur = globals::stdval::maxdur::safe;
        c.interval = 0;
        c.steps = -1;

        return;
      }

      // TODO: PPP for other Modes.

      auto* maybe_preset = SafeAt(Presets::presets, idx);
      if (maybe_preset) {
        (*maybe_preset)(b);
      } else {
        Serial.println("Unregistered Preset index");
        m = M::Idle_Init;
      }
    } break;
    default:
      break;
  }
}
