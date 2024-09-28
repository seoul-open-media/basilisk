#pragma once

#include "meta.h"

void ModeRunners::Wait(Basilisk* b) {
  auto& m = b->cmd_.mode;
  auto& c = b->cmd_.wait;

  switch (m) {
    case M::Wait: {
      if (c.exit_condition(b)) m = c.exit_to_mode;
    } break;
    default:
      break;
  }
}
