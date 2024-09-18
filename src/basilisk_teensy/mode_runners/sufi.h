#pragma once

#include "mode_runners.h"

void ModeRunners::Sufi(Basilisk* b) {
  auto& m = b->cmd_.mode;
  auto& c = b->cmd_.sufi;
  auto& pivseq = b->cmd_.pivseq;

  const uint8_t left_step = c.init_didimbal == BOOL_L ? 1 : 0;
  const uint8_t right_step = c.init_didimbal == BOOL_L ? 0 : 1;

  switch (m) {
    case M::Sufi: {
      // Serial.println("ModeRunners::Sufi");

      m = M::PivSeq_Init;
      pivseq.exit_condition = [](Basilisk* b) {
        return abs(b->imu_.GetYaw(true) - b->cmd_.sufi.tgt_yaw) < 0.1;
      };
      c.pivots[0].didimbal = c.init_didimbal;
      c.pivots[1].didimbal = !c.init_didimbal;
      c.pivots[left_step].stride = -c.stride;
      c.pivots[right_step].stride = c.stride;
      for (const uint8_t step : {0, 1}) {
        c.pivots[step].tgt_yaw = [](Basilisk* b) { return NaN; };
        c.pivots[step].bend[IDX_L] = NaN;  // c.bend[IDX_L];
        c.pivots[step].bend[IDX_R] = NaN;  // c.bend[IDX_R];
        c.pivots[step].speed = c.speed;
        c.pivots[step].acclim = c.acclim;
        c.min_durs[step] = c.min_stepdur;
        c.max_durs[step] = c.max_stepdur;
      }
      pivseq.pivots = c.pivots;
      pivseq.min_durs = c.min_durs;
      pivseq.max_durs = c.max_durs;
      pivseq.size = 2;
      pivseq.loop_begin_idx = 0;
      pivseq.steps = c.steps;
      pivseq.exit_to_mode = M::Idle_Init;
    } break;
    default:
      break;
  }
}
