#pragma once

#include "../basilisk.h"

void ExecFuncs::Em(Basilisk& b) {
  using C = Basilisk::Command::Em;
  using FSM = C::FSMState;
  auto& c = b.cmd_.em;
  auto& fsm = c.fsm_state;

  switch (fsm) {
    case FSM::Init: {
      Serial.println(F("ExecFuncs::Em(FSM::Init)"));

      for (uint8_t id = 0; id < 4; id++) {
        b.ems_.SetStrength(id, c.strength[id - 1]);
      }

      b.cmd_.mode = Basilisk::Command::Mode::None;
    } break;
    default: {
      Serial.println(F("ExecFuncs::Em(FSM::UNKNOWN)"));
    } break;
  }
}

namespace cmd_presets::em {
using C = const Basilisk::Command::Em;
using FSM = C::FSMState;

C fix_all{.fsm_state = FSM::Init,
          .strength = {EmStrength::Max,  //
                       EmStrength::Max,  //
                       EmStrength::Max,  //
                       EmStrength::Max}};

C free_all{.fsm_state = FSM::Init,
           .strength = {EmStrength::Weak,  //
                        EmStrength::Weak,  //
                        EmStrength::Weak,  //
                        EmStrength::Weak}};

}  // namespace cmd_presets::em
