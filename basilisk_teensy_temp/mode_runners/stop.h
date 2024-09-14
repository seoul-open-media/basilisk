#pragma once

#include "../basilisk.h"

void ExecFuncs::Stop(Basilisk& b) {
  using C = Basilisk::Command::Stop;
  using FSM = C::FSMState;
  auto& c = b.cmd_.stop;
  auto& fsm = c.fsm_state;

  switch (fsm) {
    case FSM::Init: {
      Serial.println(F("ExecFuncs::Stop(FSM::Init)"));

      b.CommandBoth([](Servo* s) { s.Stop(); });

      b.cmd_.mode = Basilisk::Command::Mode::None;
    } break;
    default: {
      Serial.println(F("ExecFuncs::Stop(FSM::UNKNOWN)"));
    } break;
  }
}
