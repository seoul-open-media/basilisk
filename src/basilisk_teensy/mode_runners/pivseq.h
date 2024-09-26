#pragma once

#include "mode_runners.h"

void ModeRunners::PivSeq(Basilisk* b) {
  static int cur_step;

  auto& m = b->cmd_.mode;
  auto& c = b->cmd_.pivseq;

  switch (m) {
    case M::PivSeq_Init: {
      Serial.println("ModeRunners::PivSeq(Init)");

      cur_step = 0;
      m = M::PivSeq_Step;
    } break;
    case M::PivSeq_Step: {
      Serial.print("ModeRunners::PivSeq(Step: ");
      Serial.print(cur_step);
      Serial.println(")");

      if (c.exit_condition(b) || cur_step >= c.steps) {
        m = c.exit_to_mode;
        return;
      }

      m = M::Pivot_Init;
      b->cmd_.pivot = c.pivots(b, cur_step);
      b->cmd_.pivot.exit_to_mode = M::Wait;
      b->cmd_.wait.init_time = millis();
      b->cmd_.wait.exit_condition = [](Basilisk* b) {
        return millis() - b->cmd_.wait.init_time >=
               b->cmd_.pivseq.intervals(b, cur_step);
      };
      b->cmd_.wait.exit_to_mode = M::PivSeq_Step;

      cur_step++;
    } break;
    default:
      break;
  }
}
