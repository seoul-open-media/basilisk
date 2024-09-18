#pragma once

#include "mode_runners.h"

void ModeRunners::WalkToPos(Basilisk* b) {
  auto& m = b->cmd_.mode;
  auto& c = b->cmd_.walk_to_pos;
  auto& pivseq = b->cmd_.pivseq;

  switch (m) {
    case M::WalkToPos: {
      // Serial.println("ModeRunners::WalkToPos");

      m = M::PivSeq_Init;
      pivseq.exit_condition = [](Basilisk* b) {
        return b->cmd_.walk_to_pos.tgt_pos.dist(b->lps_.GetPos()) <
               abs(b->cmd_.walk_to_pos.prox_thr);
      };
      c.pivots[0].didimbal = c.init_didimbal;
      c.pivots[1].didimbal = !c.init_didimbal;
      for (const uint8_t step : {0, 1}) {
        c.pivots[step].tgt_yaw = [](Basilisk* b) {
          return (b->cmd_.walk_to_pos.tgt_pos - b->lps_.GetPos()).arg();
        };
        c.pivots[step].bend[IDX_L] = c.bend[IDX_L];
        c.pivots[step].bend[IDX_R] = c.bend[IDX_R];
        c.pivots[step].stride = c.stride;
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
