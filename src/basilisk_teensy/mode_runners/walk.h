#pragma once

#include "mode_runners.h"

void ModeRunners::Walk(Basilisk* b) {
  // auto& m = b->cmd_.mode;
  // auto& c = b->cmd_.walk;

  // static const double phidot_std = 0.15;
  // static const double phiddot_std = 0.5;

  // switch (m) {
  //   case M::Walk_Init: {
  //     Serial.println("ModeRunners::Walk(Init)");
  //     b->Print();

  //     // Stop both Servos and reset cur_step.
  //     Serial.println(F("Stop both Servos and reset cur_step."));
  //     b->CommandBoth([](Servo* s) { s->SetStop(); });
  //     c.cur_step = 0;
  //     b->Print();

  //     // Begin initializing left foot.
  //     Serial.println(F("Begin initializing left foot."));
  //     m = M::SetMags;
  //     b->cmd_.set_mags.strengths[0] = MagnetStrength::Min;
  //     b->cmd_.set_mags.strengths[1] = MagnetStrength::Min;
  //     b->cmd_.set_mags.strengths[2] = MagnetStrength::Max;
  //     b->cmd_.set_mags.strengths[3] = MagnetStrength::Max;
  //     b->cmd_.set_mags.expected_contact[0] = false;
  //     b->cmd_.set_mags.expected_contact[1] = true;
  //     b->cmd_.set_mags.verif_thr[0] = 32;
  //     b->cmd_.set_mags.verif_thr[1] = 32;
  //     b->cmd_.set_mags.min_wait_time = 100;
  //     b->cmd_.set_mags.max_wait_time = 3000;
  //     b->cmd_.set_mags.exit_to_mode = M::SetPhis_Init;
  //     b->cmd_.set_phis.SetPhis(-0.25 - c.bend_l, NaN);
  //     b->cmd_.set_phis.SetPhiDots(phidot_std, 0.0);
  //     b->cmd_.set_phis.SetPhiDDots(phiddot_std, 0.0);
  //     b->cmd_.set_phis.SetDampThr(0.1);
  //     b->cmd_.set_phis.exit_to_mode = M::Walk_Step;
  //     b->Print();
  //   } break;
  //   case M::Walk_Step: {
  //     Serial.println("ModeRunners::Walk(Step)");
  //     b->Print();

  //     Serial.print("Current step: ");
  //     Serial.println(c.cur_step);
  //     if (c.cur_step >= c.steps) {
  //       Serial.println("All steps are stepped. Enter Idle Mode.");
  //       m = M::Idle_Init;
  //       return;
  //     }

  //     if (c.step_lr) {
  //       Serial.println(F("Move left foot."));

  //       m = M::SetMags;
  //       b->cmd_.set_mags.strengths[0] = MagnetStrength::Min;
  //       b->cmd_.set_mags.strengths[1] = MagnetStrength::Min;
  //       b->cmd_.set_mags.strengths[2] = MagnetStrength::Max;
  //       b->cmd_.set_mags.strengths[3] = MagnetStrength::Max;
  //       b->cmd_.set_mags.expected_contact[0] = false;
  //       b->cmd_.set_mags.expected_contact[1] = true;
  //       b->cmd_.set_mags.verif_thr[0] = 32;
  //       b->cmd_.set_mags.verif_thr[1] = 32;
  //       b->cmd_.set_mags.min_wait_time = 100;
  //       b->cmd_.set_mags.max_wait_time = 3000;
  //       b->cmd_.set_mags.exit_to_mode = M::SetPhis_Init;
  //       b->cmd_.set_phis.SetPhis(-0.25 - c.bend_l - c.stride,
  //                               -0.25 - c.bend_r - c.stride);
  //       b->cmd_.set_phis.SetPhiDots(phidot_std, phidot_std);
  //       b->cmd_.set_phis.SetPhiDDots(phiddot_std, phiddot_std);
  //       b->cmd_.set_phis.SetDampThr(0.1);
  //       b->cmd_.set_phis.exit_to_mode = M::Walk_Step;
  //     } else {
  //       Serial.println(F("Move right foot."));

  //       m = M::SetMags;
  //       b->cmd_.set_mags.strengths[0] = MagnetStrength::Max;
  //       b->cmd_.set_mags.strengths[1] = MagnetStrength::Max;
  //       b->cmd_.set_mags.strengths[2] = MagnetStrength::Min;
  //       b->cmd_.set_mags.strengths[3] = MagnetStrength::Min;
  //       b->cmd_.set_mags.expected_contact[0] = true;
  //       b->cmd_.set_mags.expected_contact[1] = false;
  //       b->cmd_.set_mags.verif_thr[0] = 32;
  //       b->cmd_.set_mags.verif_thr[1] = 32;
  //       b->cmd_.set_mags.min_wait_time = 100;
  //       b->cmd_.set_mags.max_wait_time = 3000;
  //       b->cmd_.set_mags.exit_to_mode = M::SetPhis_Init;
  //       b->cmd_.set_phis.SetPhis(-0.25 - c.bend_l + c.stride,
  //                               -0.25 - c.bend_r + c.stride);
  //       b->cmd_.set_phis.SetPhiDots(phidot_std, phidot_std);
  //       b->cmd_.set_phis.SetPhiDDots(phiddot_std, phiddot_std);
  //       b->cmd_.set_phis.SetDampThr(0.1);
  //       b->cmd_.set_phis.exit_to_mode = M::Walk_Step;
  //     }
  //     b->Print();

  //     c.cur_step++;
  //     c.step_lr = !c.step_lr;
  //   } break;
  //   default:
  //     break;
  // }
}
