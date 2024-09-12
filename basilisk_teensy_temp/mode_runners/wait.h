#pragma once

#include "../basilisk.h"

void ExecFuncs::Wait(Basilisk* b) {
  using C = Basilisk::Command::Wait;
  using FSM = C::FSMState;
  auto& c = b.cmd_.wait;
  auto& fsm = c.fsm_state;

  switch (fsm) {
    case FSM::Wait: {
      if (millis() - c.init_time >= c.duration) {
      }
    } break;
    default:
      break;
  }
}
