#pragma once

#include "mode_runners.h"

void ModeRunners::Walk(Basilisk* b) {
  auto& m = b->cmd_.mode;
  auto& c = b->cmd_.walk;

  switch (m) {
    case M::Walk_Init: {
      Serial.println("ModeRunners::Walk(Init)");
      b->Print();

      // Stop both Servos.
      Serial.println(F("Stop both Servos."));
      b->CommandBoth([](Servo& s) { s.SetStop(); });
      b->Print();

      // Reset current_step.
      Serial.println(F("Reset current_step."));
      c.current_step = 0;
      b->Print();

      // Begin initializing left foot.
      Serial.println(F("Begin initializing left foot."));
      m = M::SetMags;
      b->cmd_.set_mags.strengths[0] = MagnetStrength::Min;
      b->cmd_.set_mags.strengths[1] = MagnetStrength::Min;
      b->cmd_.set_mags.strengths[2] = MagnetStrength::Max;
      b->cmd_.set_mags.strengths[3] = MagnetStrength::Max;
      b->cmd_.set_mags.expected_contact[0] = false;
      b->cmd_.set_mags.expected_contact[1] = true;
      b->cmd_.set_mags.exit_mode = M::SetPhi;
      b->cmd_.set_phi.SetPhis(-0.25 - c.eightwalk_l, NaN);
      b->cmd_.set_phi.exit_to_mode = M::Walk_InitPhiR;
      b->Print();
    } break;
    case M::Walk_InitPhiR: {
      Serial.println("ModeRunners::Walk(InitPhiR)");
      b->Print();

      Serial.println(F("Begin initializing right foot."));
      m = M::SetMags;
      b->cmd_.set_mags.strengths[0] = MagnetStrength::Max;
      b->cmd_.set_mags.strengths[1] = MagnetStrength::Max;
      b->cmd_.set_mags.strengths[2] = MagnetStrength::Min;
      b->cmd_.set_mags.strengths[3] = MagnetStrength::Min;
      b->cmd_.set_mags.expected_contact[0] = true;
      b->cmd_.set_mags.expected_contact[1] = false;
      b->cmd_.set_mags.exit_mode = M::SetPhi;
      b->cmd_.set_phi.SetPhis(NaN, -0.25 - c.eightwalk_r);
      b->cmd_.set_phi.exit_to_mode = M::Walk_Step;
      b->Print();
    } break;
    case M::Walk_Step: {
      Serial.println("ModeRunners::Walk(Step)");
      b->Print();

      Serial.print("Current step: ");
      Serial.println(c.current_step);
      if (c.current_step >= c.steps) {
        Serial.println("All steps are stepped. Enter Idle Mode.");
        m = M::Idle_Init;
        return;
      }

      if (c.phase) {
        Serial.println(F("Move left foot."));

        m = M::SetMags;
        b->cmd_.set_mags.strengths[0] = MagnetStrength::Min;
        b->cmd_.set_mags.strengths[1] = MagnetStrength::Min;
        b->cmd_.set_mags.strengths[2] = MagnetStrength::Max;
        b->cmd_.set_mags.strengths[3] = MagnetStrength::Max;
        b->cmd_.set_mags.expected_contact[0] = false;
        b->cmd_.set_mags.expected_contact[1] = true;
        b->cmd_.set_mags.exit_mode = M::SetPhi;
        b->cmd_.set_phi.SetPhis(-0.25 - c.eightwalk_l - c.stride,
                                -0.25 - c.eightwalk_r - c.stride);
        b->cmd_.set_phi.exit_to_mode = M::Walk_Step;
        b->Print();
      } else {
        Serial.println(F("Move right foot."));

        m = M::SetMags;
        b->cmd_.set_mags.strengths[0] = MagnetStrength::Max;
        b->cmd_.set_mags.strengths[1] = MagnetStrength::Max;
        b->cmd_.set_mags.strengths[2] = MagnetStrength::Min;
        b->cmd_.set_mags.strengths[3] = MagnetStrength::Min;
        b->cmd_.set_mags.expected_contact[0] = true;
        b->cmd_.set_mags.expected_contact[1] = false;
        b->cmd_.set_mags.exit_mode = M::SetPhi;
        b->cmd_.set_phi.SetPhis(-0.25 - c.eightwalk_l + c.stride,
                                -0.25 - c.eightwalk_r + c.stride);
        b->cmd_.set_phi.exit_to_mode = M::Walk_Step;
        b->Print();
      }

      c.current_step++;
      c.phase = !c.phase;
    } break;
    default:
      break;
  }
}
