#pragma once

#include "../basilisk.h"

void ExecFuncs::Gee(Basilisk& b) {
  using C = Basilisk::Command::Gee;
  using FSM = C::FSMState;
  auto& c = b.cmd_.gee;
  auto& fsm = c.fsm_state;

  switch (c.fsm_state) {
    case FSM::Init: {
      Serial.println(F("ExecFuncs::Gee(FSM::Init)"));
      b.Print();

      // Fix left foot and free right foot
      b.ems_.FixLeft();
      b.ems_.FreeRight();
      b.Print();

      // Reset current_step.
      c.current_step = 0;

      // Begin moving to zero pose and enter FSM::Wait.
      b.CommandBoth([](Servo* s) { s.Position(-0.25); });
      c.fsm_state = FSM::Wait;
    } break;
    case FSM::Wait: {
      Serial.println(F("ExecFuncs::Gee(FSM::Wait)"));
      b.Print();

      // Resume step when complete
      // or enter FSM::Complete state when all steps are done.
      if (b.BothComplete(4)) {
        if (c.current_step < c.steps) {
          c.fsm_state = FSM::Step;
        } else {
          b.CommandBoth([](Servo* s) { s.Stop(); });
          b.cmd_.mode = Basilisk::Command::Mode::None;
        }
      }
    } break;
    case FSM::Step: {
      Serial.println(F("ExecFuncs::Gee(FSM::Step)"));
      if (c.phase) {
        // Fix toes and free ankles.
        b.ems_.FixToes();
        b.ems_.FreeAnkles();

        // Shear.
        b.CommandBoth([&](Servo& s) { s.Position(-0.25 - c.stride); });
      } else {
        // Fix ankles and free toes.
        b.ems_.FixAnkles();
        b.ems_.FreeToes();

        // Shear.
        b.CommandBoth([&](Servo& s) { s.Position(-0.25 + c.stride); });
      }

      // Update phase and current_step and enter FSM::Wait state.
      c.phase = !c.phase;
      c.current_step++;
      c.fsm_state = FSM::Wait;
    } break;
    default:
      break;
  }
}

namespace cmd_presets::gee {
using C = const Basilisk::Command::Gee;
using FSM = C::FSMState;

C right{0.125, 8};

C left{-0.125, 8};

}  // namespace cmd_presets::gee
