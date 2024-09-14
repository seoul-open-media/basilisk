#pragma once

#include "mode_runners.h"

void ModeRunners::Face(Basilisk* b) {
  auto& m = b->cmd_.mode;
  auto& c = b->cmd_.face;

  switch (m) {
    case M::Face: {
      Serial.println("ModeRunners::Face()");

      if (!c.phase) {
        c.phase = true;
      } else {
        c.phase = false;
        m = c.exit_to_mode;
      }
    } break;
    default:
      break;
  }
}
