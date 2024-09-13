#pragma once

#include "mode_runners.h"

void ModeRunners::Idle(Basilisk* b) {
  using M = Basilisk::Command::Mode;
  auto& m = b->cmd_.mode;

  switch (b->cmd_.mode) {
    case M::Idle_Init: {
      Serial.println("ModeRunners::Idle(Init)");
      b->CommandBoth([](Servo& s) { s.SetStop(); });
      m = M::Idle_Nop;
    } break;
    case M::Idle_Nop: {
    } break;
    default:
      break;
  }
}
