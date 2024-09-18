#pragma once

#include "mode_runners.h"

void ModeRunners::PivSeq(Basilisk* b) {
  auto& m = b->cmd_.mode;
  auto& c = b->cmd_.pivseq;

  switch (m) {
    case M::PivSeq_Init: {
      Serial.println("ModeRunners::PivSeq(Init)");

      c.cur_step = 0;
      c.cur_idx = 0;
      m = M::PivSeq_Step;
    } break;
    case M::PivSeq_Step: {
      Serial.print("ModeRunners::PivSeq(Step: ");
      Serial.print(c.cur_step);
      Serial.println(")");

      if (c.exit_condition(b) || c.cur_step >= c.steps) {
        m = c.exit_to_mode;
        return;
      }

      if (c.cur_idx >= c.size) {
        c.cur_idx = c.loop_begin_idx;
      }

      m = M::Pivot_Init;
      b->cmd_.pivot = c.pivots[c.cur_idx];
      b->cmd_.pivot.min_dur = c.min_durs[c.cur_idx];
      b->cmd_.pivot.max_dur = c.max_durs[c.cur_idx];
      b->cmd_.pivot.exit_to_mode = M::PivSeq_Step;

      c.cur_step++;
      c.cur_idx++;
    } break;
    default:
      break;
  }
}
