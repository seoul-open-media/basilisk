#pragma once

#include "../basilisk.h"

void ExecFuncs::SetRho(Basilisk& b) {
  using C = Basilisk::Command::SetRho;
  using FSM = C::FSMState;
  auto& c = b.cmd_.set_rho;
  auto& fsm = c.fsm_state;

  switch (fsm) {
    case FSM::Init: {
      Serial.println(F("ExecFuncs::SetRho(FSM::Init)"));

      b.l_.Position(c.tgt_rho_l);
      b.r_.Position(c.tgt_rho_r);

      fsm = FSM::Wait;
    } break;
    case FSM::Wait: {
      Serial.println(F("ExecFuncs::SetRho(FSM::Wait)"));

      // Query is done at Executer::Run before entering FSM-based stage.
      if (b.BothComplete(4)) {
        b.CommandBoth([](Servo& s) { s.Stop(); });

        b.cmd_.mode = Basilisk::Command::Mode::None;
      }
    } break;
    default: {
      Serial.println(F("ExecFuncs::SetRho(FSM::UNKNOWN)"));
    } break;
  }
}

namespace cmd_presets::set_rho {
using C = const Basilisk::Command::SetRho;
using FSM = C::FSMState;

C m025{.fsm_state = FSM::Init, .tgt_rho_l = -0.25, .tgt_rho_r = -0.25};

C zero{.fsm_state = FSM::Init, .tgt_rho_l = 0.0, .tgt_rho_r = 0.0};

}  // namespace cmd_presets::set_rho
