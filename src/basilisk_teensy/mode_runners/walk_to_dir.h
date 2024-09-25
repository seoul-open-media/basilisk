#pragma once

#include "mode_runners.h"

void ModeRunners::WalkToDir(Basilisk* b) {
  auto& m = b->cmd_.mode;
  auto& c = b->cmd_.walk_to_dir;
  auto& pivseq = b->cmd_.pivseq;

  switch (m) {
    case M::WalkToDir: {
      // Serial.println("ModeRunners::WalkToDir");

      m = M::Walk;
      auto& w = b->cmd_.walk;
      w.init_didimbal = c.init_didimbal;
      for (uint8_t f : IDX_LR) {
        w.tgt_yaw[f] = [](Basilisk* b) {
          auto& c = b->cmd_.walk_to_dir;
          return c.tgt_yaw;
        };
        w.stride[f] = [](Basilisk* b) {
          auto& c = b->cmd_.walk_to_dir;
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
      w.exit_condition = [](Basilisk* b) { return false; };
    } break;
    default:
      break;
  }
}
