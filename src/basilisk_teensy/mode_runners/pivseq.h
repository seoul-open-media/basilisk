#pragma once

#include "mode_runners.h"

void ModeRunners::PivSeq(Basilisk* b) {
  auto& m = b->cmd_.mode;
  auto& c = b->cmd_.pivseq;

  switch (m) {
    case M::PivSeq_Init: {
      // Serial.println("ModeRunners::PivSeq(Init)");
      c.cur_step = 0;
      c.cur_idx = 0;
      m = M::PivSeq_Step;
    } break;
    case M::PivSeq_Step: {
      // Serial.print("ModeRunners::PivSeq(Step: ");
      // Serial.print(c.cur_step);
      // Serial.print(", ");
      // Serial.print(c.cur_idx);
      // Serial.println(")");

      if (c.exit_condition(b) || c.cur_step >= c.steps) {
        m = c.exit_to_mode;
        return;
      }
      if (c.cur_idx >= c.loop_end_idx) {
        c.cur_idx = c.loop_begin_idx;
      }

      m = M::Pivot_Init;
      b->cmd_.pivot = c.pivots(b, c.cur_idx);
      b->cmd_.pivot.exit_to_mode = M::Wait;
      b->cmd_.wait.exit_condition = [](Basilisk* b) {
        auto& ps = b->cmd_.pivseq;
        return millis() - b->cmd_.wait.init_time > ps.intervals(b, ps.cur_idx);
      };
      b->cmd_.wait.exit_to_mode = M::PivSeq_Step;
      b->cmd_.wait.init_time = millis();

      c.cur_step++;
      c.cur_idx++;
    } break;
    default:
      break;
  }
}
