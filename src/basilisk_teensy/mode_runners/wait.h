#pragma once

#include "mode_runners.h"

void ModeRunners::Wait(Basilisk* b) {
  auto& m = b->cmd_.mode;

  switch (m) {
    case M::Wait: {
      Serial.println("ModeRunners::Wait()");
      if (!b->cmd_.wait.exit_condition(b)) return;
      m = b->cmd_.wait.exit_to_mode;
    } break;
    default:
      break;
  }
}
