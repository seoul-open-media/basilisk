#pragma once

#include "../basilisk.h"

// 200ms blocking delay, necessary for stability.
void ExecFuncs::DExactM025(Basilisk& b) {
  using C = Basilisk::Command::DExactM025;
  using FSM = C::FSMState;
  auto& c = b.cmd_.d_exact_m025;
  auto& fsm = c.fsm_state;

  switch (fsm) {
    case FSM::Init: {
      Serial.println(F("ExecFuncs::DExactM025(FSM::Init)"));

      b.CommandBoth([](Servo& s) {
        s.Stop();
        delay(50);
        s.d(F("d exact -0.25"));
        delay(50);
        s.Stop();
      });

      b.cmd_.mode = Basilisk::Command::Mode::None;
    } break;
    default: {
      Serial.println(F("ExecFuncs::DExactM025(FSM::UNKNOWN)"));
    } break;
  }
}
