#pragma once

#include "meta.h"

void ModeRunners::Free(Basilisk* b) {
  auto& m = b->cmd_.mode;

  switch (m) {
    case M::Free: {
      b->CommandBoth([](Servo* s) { s->SetStop(); });
      b->mags_.ReleaseAll();
      m = M::Wait;
      b->cmd_.wait.init_time = millis();
      b->cmd_.wait.exit_condition = [](Basilisk* b) {
        return millis() - b->cmd_.wait.init_time > 3000;
      };
      b->cmd_.wait.exit_to_mode = M::Idle_Init;
    } break;
    default:
      break;
  }
}
