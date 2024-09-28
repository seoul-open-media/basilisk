#pragma once

#include "meta.h"

void ModeRunners::PivSpin(Basilisk* b) {
  auto& m = b->cmd_.mode;
  auto& c = b->cmd_.piv_spin;
  auto& ps = b->cmd_.pivseq;

  switch (m) {
    case M::PivSpin: {
      Serial.println("ModeRunners::PivSpin");

      m = M::PivSeq_Init;
      ps.pivots = [](Basilisk* b, int idx) {
        auto& c = b->cmd_.piv_spin;
        Basilisk::Command::Pivot p;
        p.didimbal = c.didimbal;
        p.tgt_yaw = [](Basilisk*) { return NaN; };
        p.stride = c.stride;
        for (const uint8_t f : IDX_LR) p.bend[f] = c.bend[f];
        p.speed = c.speed;
        p.acclim = c.acclim;
        p.min_dur = c.min_stepdur;
        p.max_dur = c.max_stepdur;
        p.exit_condition = [](Basilisk* b) {
          auto& c = b->cmd_.piv_spin;
          return abs(b->imu_.GetYaw(true) - c.dest_yaw) < c.exit_thr;
        };
        return p;
      };
      ps.intervals = [](Basilisk* b, int idx) {
        auto& c = b->cmd_.piv_spin;
        return c.interval;
      };
      ps.steps = c.steps;
      ps.exit_condition = [](Basilisk* b) {
        auto& c = b->cmd_.piv_spin;
        return abs(b->imu_.GetYaw(true) - c.dest_yaw) < c.exit_thr;
      };
      ps.exit_to_mode = M::Idle_Init;
    } break;
    default:
      break;
  }
}
