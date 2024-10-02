#pragma once

#include "../basilisk.h"

void ExecFuncs::Walk(Basilisk& b) {
  using C = Basilisk::Command::Walk;
  using FSM = C::FSMState;
  auto& c = b.cmd_.walk;
  auto& fsm = c.fsm_state;

  switch (fsm) {
    case FSM::Init: {
      Serial.println(F("ExecFuncs::Walk(FSM::Init)"));
      b.Print();

      // Stop both Servos.
      Serial.println(F("Stop both Servos."));
      b.CommandBoth([](Servo* s) { s.Stop(); });
      b.Print();

      // Reset current_step.
      Serial.println(F("Reset current_step."));
      c.current_step = 0;
      b.Print();

      // Begin initializing left foot and enter FSM::WaitInitLeft.
      Serial.println(
          F("Begin initializing left foot and enter FSM::WaitInitLeft."));
      b.ems_.FreeLeft();
      b.ems_.FixRight();
      b.r_.Position(NaN);
      b.l_.Position(-0.25 - c.eightwalk_l);
      c.fsm_state = FSM::WaitInitLeft;
      b.Print();
    } break;
    case FSM::WaitInitLeft: {
      Serial.println(F("ExecFuncs::Walk(FSM::WaitInitLeft)"));
      b.Print();

      // When left foot initialization is complete,
      // begin initializing right foot and enter FSM::WaitInitRight.
      if (b.BothComplete(4)) {
        Serial.println(F("Left foot initialization complete."));
        Serial.println(
            F("Begin initializing right foot and enter FSM::WaitInitRight."));
        b.ems_.FixLeft();
        b.ems_.FreeRight();
        b.l_.Position(NaN);
        b.r_.Position(-0.25 - c.eightwalk_r);
        c.fsm_state = FSM::WaitInitRight;
        b.Print();
      }
    } break;
    case FSM::WaitInitRight: {
      Serial.println(F("ExecFuncs::Walk(FSM::WaitInitLeft)"));
      b.Print();

      // When right foot initialization is complete, enter FSM::Move.
      if (b.BothComplete(4)) {
        Serial.println(F("Right foot initialization complete."));
        Serial.println(F("Enter FSM::Move."));

        c.fsm_state = FSM::Move;
        b.Print();
      }
    } break;
    case FSM::Move: {
      Serial.println(F("ExecFuncs::Walk(FSM::Move)"));
      b.Print();

      if (c.phase) {
        Serial.println(F("Move left foot."));

        // Fix right foot and free left foot.
        Serial.println(F("Fix right foot and free left foot"));
        b.ems_.FixRight();
        b.ems_.FreeLeft();

        // temp
        delay(500);

        b.Print();

        // Control phis.
        Serial.println(F("Control phis"));
        b.l_.Position(-0.25 - c.eightwalk_l - c.stride);
        b.r_.Position(-0.25 - c.eightwalk_r - c.stride);
        b.Print();
      } else {
        Serial.println(F("Move right foot."));

        // Fix left foot and free right foot.
        Serial.println(F("Fix left foot and free right foot."));

        b.ems_.FixLeft();
        b.ems_.FreeRight();

        // temp
        delay(500);

        b.Print();

        // Control phis.
        Serial.println(F("Control phis"));
        b.l_.Position(-0.25 - c.eightwalk_l + c.stride);
        b.r_.Position(-0.25 - c.eightwalk_r + c.stride);
        b.Print();
      }

      // Update phase and current_step and jump to FSM::WaitMove.
      Serial.println(
          F("Update phase and current_step and jump to FSM::WaitMove."));
      c.current_step++;
      c.phase = !c.phase;
      c.fsm_state = FSM::WaitMove;
      b.Print();
    } break;
    case FSM::WaitMove: {
      Serial.println(F("ExecFuncs::Walk(FSM::WaitMove)"));
      b.Print();

      // Resume to FSM::Move state
      // or return to Mode::None if done walking all steps.
      if (b.BothComplete(4)) {
        if (c.current_step < c.steps) {
          c.fsm_state = FSM::Move;
        } else {
          b.CommandBoth([](Servo* s) { s.Stop(); });
          b.cmd_.mode = Basilisk::Command::Mode::None;
        }
      }
      b.Print();
    } break;
    default: {
      Serial.println(F("ExecFuncs::Walk(FSM::UNKNOWN)"));
    } break;
  }
}

namespace cmd_presets::walk {
using C = const Basilisk::Command::Walk;
using FSM = C::FSMState;

C square{0.125, 0.0, 0.0, 8, false};

C catwalk{0.25, 0.0, 0.0, 4, false};

C babywalk{10.0 / 360.0, 0.0, 0.0, 32, false};

C eightwalk{0.125, 0.125, -0.125, 8, false};

}  // namespace cmd_presets::walk
