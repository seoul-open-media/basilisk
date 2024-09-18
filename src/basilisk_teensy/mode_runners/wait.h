#pragma once

#include "mode_runners.h"

void ModeRunners::Wait(Basilisk* b) {
  auto& m = b->cmd_.mode;
  auto& c = b->cmd_.wait;

  switch (m) {
    case M::Wait: {
      Serial.println("ModeRunners::Wait");

      if (c.exit_condition(b)) m = c.exit_to_mode;
    } break;
    default:
      break;
  }
}
