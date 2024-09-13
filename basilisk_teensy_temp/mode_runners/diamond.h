#pragma once

#include "../basilisk.h"

void ExecFuncs::Diamond(Basilisk& b) {
  using C = Basilisk::Command::Diamond;
  using FSM = C::FSMState;
  auto& c = b.cmd_.diamond;
  auto& fsm = c.fsm_state;

  switch (fsm) {
    case FSM::Init: {
      Serial.println(F("ExecFuncs::Diamond(FSM::Init)"));
      b.Print();

      // Reset current_step to 0.
      Serial.println(F("Reset current_step to 0."));
      c.current_step = 0;

      // Fix left foot and free right foot.
      Serial.println(F("Fix left foot and free right foot."));
      b.ems_.FixLeft();
      b.ems_.FreeRight();
      b.Print();

      // Begin moving to standby position and enter FSM::Wait.
      Serial.println(
          F("Begin moving to standby position and enter FSM::Wait."));
      b.CommandBoth([&](Servo& s) { s.Position(-0.25); });
      c.fsm_state = FSM::Wait;
      b.Print();
    } break;
    case FSM::Wait: {
      Serial.println(F("ExecFuncs::Diamond(FSM::Wait)"));
      b.Print();

      // Resume to FSM::Step state when complete.
      if (b.BothComplete(4)) {
        Serial.println(F("Wait complete; resume to FSM::Step."));
        c.fsm_state = FSM::Step;
        b.Print();
      }
    } break;
    case FSM::Step: {
      Serial.println(F("ExecFuncs::Diamond(FSM::Step)"));
      b.Print();

      Serial.print(F("Current step: "));
      Serial.println(c.current_step);

      if (c.current_step % 2 == 0) {
        // Fix left foot and free right foot.
        b.ems_.FixLeft();
        b.ems_.FixRight();
      } else {
        // Fix right foot and free left foot.
        b.ems_.FixRight();
        b.ems_.FreeLeft();
      }
      b.Print();

      // Command both rhos to target.
      Serial.print(F("Command both rhos to target: "));
      Serial.println(c.tgt_rho(c.current_step));
      b.CommandBoth(
          [&](Servo& s) { s.Position(c.tgt_rho(c.current_step)); });
      b.Print();

      // Increment current_step and enter FSM::Wait state.
      c.current_step++;
      if (c.current_step == 4) {
        c.current_step = 0;
      }
      c.fsm_state = FSM::Wait;
      b.Print();
    } break;
    default:
      break;
  }
}

namespace cmd_presets::diamond {
using C = const Basilisk::Command::Diamond;
using FSM = C::FSMState;

C square(0.125);

}  // namespace cmd_presets::diamond
