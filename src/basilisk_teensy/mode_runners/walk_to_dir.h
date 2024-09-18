#pragma once

#include "mode_runners.h"

void ModeRunners::WalkToDir(Basilisk* b) {
  auto& m = b->cmd_.mode;
  auto& c = b->cmd_.walk_to_dir;
  auto& pivseq = b->cmd_.pivseq;

  switch (m) {
    case M::WalkToDir: {
      Serial.println("ModeRunners::WalkToDir");

      if (isnan(c.tgt_yaw)) c.tgt_yaw = b->imu_.GetYaw(true);

      m = M::PivSeq_Init;
      pivseq.exit_condition = [](Basilisk*) { return false; };
      c.pivots[0].didimbal = c.init_didimbal;
      c.pivots[1].didimbal = !c.init_didimbal;
      for (const uint8_t i : {0, 1}) {
        c.pivots[i].tgt_yaw = c.tgt_yaw;
        c.pivots[i].bend[IDX_L] = c.bend[IDX_L];
        c.pivots[i].bend[IDX_R] = c.bend[IDX_R];
        c.pivots[i].stride = c.stride;
        c.pivots[i].speed = c.speed;
        c.pivots[i].accel = c.accel;
      }
      pivseq.pivots = c.pivots;
      c.min_durs[0] = c.min_stepdur;
      c.min_durs[1] = c.min_stepdur;
      pivseq.min_durs = c.min_durs;
      c.max_durs[0] = c.max_stepdur;
      c.max_durs[1] = c.max_stepdur;
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
