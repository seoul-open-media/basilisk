#pragma once

#include "mode_runners.h"

void ModeRunners::Idle(Basilisk* b) {
  auto& m = b->cmd_.mode;
  static bool nop_init = false;

  switch (m) {
    case M::Idle_Init: {
      Serial.println("ModeRunners::Idle(Init)");
      b->CommandBoth([](Servo* s) { s->SetStop(); });
      b->mags_.FixAll();
      m = M::Idle_Nop;
      nop_init = true;
    } break;
    case M::Idle_Nop: {
      if (nop_init) {
        Serial.println("ModeRunners::Idle(Nop)");
        nop_init = false;
      }
    } break;
    default:
      break;
  }
}
