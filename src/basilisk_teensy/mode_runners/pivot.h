#pragma once

#include "mode_runners.h"

void ModeRunners::Pivot(Basilisk* b) {
  auto& m = b->cmd_.mode;
  auto& c = b->cmd_.pivot;

  switch (m) {
    case M::Pivot_Init: {
      Serial.println("ModeRunners::Face(Init)");
      b->Print();

      // Set didimbal.
      Serial.println("Set didimbal");
      m = M::SetMags;
      b->cmd_.set_mags.strengths[0] = Bool2MS(c.didimbal == BOOL_L);
      b->cmd_.set_mags.strengths[1] = Bool2MS(c.didimbal == BOOL_L);
      b->cmd_.set_mags.strengths[2] = Bool2MS(c.didimbal == BOOL_R);
      b->cmd_.set_mags.strengths[3] = Bool2MS(c.didimbal == BOOL_R);
      b->cmd_.set_mags.expected_contact[0] = (c.didimbal = BOOL_L);
      b->cmd_.set_mags.expected_contact[1] = (c.didimbal = BOOL_R);
      // b->cmd_.set_mags.verif_thr = 32;
    } break;
    case M::Pivot_Kick: {
      Serial.println("ModeRunners::Face(Kick)");
      m = c.exit_to_mode;
    } break;
    default:
      break;
  }
}
