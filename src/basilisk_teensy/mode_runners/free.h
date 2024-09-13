#pragma once

#include "mode_runners.h"

void ModeRunners::Free(Basilisk* b) {
  using M = Basilisk::Command::Mode;
  auto& m = b->cmd_.mode;

  switch (b->cmd_.mode) {
    case M::Free: {
      Serial.println("ModeRunners::Free()");
      b->CommandBoth([](Servo& s) { s.SetStop(); });
      b->mags_.FreeAll();
      b->cmd_.wait.exit_condition = [](Basilisk&) {
        static const auto init_time = millis();
        return millis() - init_time > 3000;
      };
      b->cmd_.wait.exit_to_mode = M::Idle_Init;
      m = M::Wait;
    } break;
    default:
      break;
  }
}
