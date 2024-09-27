#pragma once

#include "meta.h"

void ModeRunners::Sufi(Basilisk* b) {
  auto& m = b->cmd_.mode;
  auto& c = b->cmd_.sufi;
  auto& w = b->cmd_.walk;

  switch (m) {
    case M::Sufi: {
      // Serial.println("ModeRunners::Sufi");

      m = M::Walk;
      w.init_didimbal = c.init_didimbal;
      for (uint8_t f : IDX_LR) {
        w.tgt_yaw[f] = [](Basilisk* b) { return NaN; };
        w.bend[f] = c.bend[f];
        w.speed[f] = c.speed;
        w.acclim[f] = c.acclim;
        w.min_stepdur[f] = c.min_stepdur;
        w.max_stepdur[f] = c.max_stepdur;
        w.interval[f] = c.interval;
      }
      w.stride[IDX_L] = [](Basilisk* b) {
        auto& c = b->cmd_.sufi;
        return c.stride;
      };
      w.stride[IDX_R] = [](Basilisk* b) {
        auto& c = b->cmd_.sufi;
        return -c.stride;
      };
      w.steps = c.steps;
      w.exit_condition = [](Basilisk* b) {
        auto& c = b->cmd_.sufi;
        return abs(b->imu_.GetYaw(true) - c.dest_yaw) < c.exit_thr;
      };
    } break;
    default:
      break;
  }
}
