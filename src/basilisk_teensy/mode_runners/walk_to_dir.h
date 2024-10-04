#pragma once

#include "meta.h"

namespace walk_to_dir {
double tgt_yaw;
}  // namespace walk_to_dir

void ModeRunners::WalkToDir(Basilisk* b) {
  auto& m = b->cmd_.mode;
  auto& c = b->cmd_.walk_to_dir;
  auto& w = b->cmd_.walk;

  switch (m) {
    case M::WalkToDir: {
      m = M::Walk;
      w.init_didimbal = c.init_didimbal;

      for (uint8_t f : IDX_LR) {
        if (isnan(c.tgt_yaw)) {
          walk_to_dir::tgt_yaw = b->imu_.GetYaw(true);
          w.tgt_yaw[f] = [](Basilisk* b) { return walk_to_dir::tgt_yaw; };
        } else {
          w.tgt_yaw[f] = [](Basilisk* b) {
            const auto& c = b->cmd_.walk_to_dir;
            return c.tgt_yaw;
          };
        }
        w.stride[f] = [](Basilisk* b) {
          const auto& c = b->cmd_.walk_to_dir;
          return c.stride;
        };
        w.bend[f] = c.bend[f];
        w.speed[f] = c.speed;
        w.acclim[f] = c.acclim;
        w.min_stepdur[f] = c.min_stepdur;
        w.max_stepdur[f] = c.max_stepdur;
        w.interval[f] = c.interval;
      }
      w.steps = c.steps;
      w.exit_condition = [](Basilisk* b) { return !b->lps_.Bound(); };
    } break;
    default:
      break;
  }
}
