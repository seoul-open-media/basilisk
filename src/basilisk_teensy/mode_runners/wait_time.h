#pragma once

#include "../basilisk.h"

void ExecFuncs::WaitTime(Basilisk& b) {
  using C = Basilisk::Command::WaitTime;
  using FSM = C::FSMState;
  auto& c = b.cmd_.wait_time;
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
