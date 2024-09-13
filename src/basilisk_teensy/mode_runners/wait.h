#pragma once

#include "mode_runners.h"

void ModeRunners::Wait(Basilisk* b) {
  using M = Basilisk::Command::Mode;
  auto& m = b->cmd_.mode;

  switch (b->cmd_.mode) {
    case M::Wait: {
      Serial.println("ModeRunners::Wait()");
      if (!b->cmd_.wait.exit_condition(*b)) return;
      m = b->cmd_.wait.exit_to_mode;
    } break;
    default:
      break;
  }
}
