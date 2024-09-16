#pragma once

#include "mode_runners.h"

void ModeRunners::SetPhis(Basilisk* b) {
  auto& m = b->cmd_.mode;
  auto& c = b->cmd_.set_phis;

  switch (m) {
    case M::SetPhis_Init: {
      Serial.println("ModeRunners::SetPhis(Init)");

      bool exit[2];

      for (uint8_t i = 0; i < 2; i++) {
        auto* s = b->lr_[i];

        const double tgt_outpos = c.tgt_phi[i];
        double tgt_outvel = c.tgt_phidot[i];
        const double tgt_outacc = c.tgt_phiddot[i];

        double tgt_rtrvel;
        double tgt_rtracc;

        if (isnan(tgt_outpos)) {
          // Velocity control with no target position is NOT allowed.
          tgt_rtrvel = 0.0;
          tgt_rtracc = 0.0;
          exit[i] = true;
        } else {
          const auto cur_outpos = s->GetReply().abs_position;
          const auto tgt_delta_outpos = tgt_outpos - cur_outpos;

          if (abs(tgt_delta_outpos) < c.fix_thr) {
            // Target phi is reached.
            tgt_rtrvel = 0.0;
            tgt_rtracc = 0.0;
            exit[i] = true;
          } else {
            tgt_outvel *= signedpow(
                constrain(tgt_delta_outpos / c.damp_thr, -1.0, 1.0), 1.5);
            tgt_rtrvel = b->gr_ * tgt_outvel;
            tgt_rtracc = b->gr_ * tgt_outacc;
            exit[i] = false;
          }
        }

        s->SetPosition([&] {
          auto pm_cmd = *b->pm_cmd_template_;
          pm_cmd.position = NaN;
          pm_cmd.velocity = tgt_rtrvel;
          pm_cmd.accel_limit = tgt_rtracc;
          return pm_cmd;
        }());
      }

      if (exit[0] && exit[1]) {
        m = M::Wait;
        b->cmd_.wait.init_time = millis();
        b->cmd_.wait.exit_condition = [](Basilisk* b) {
          return millis() - b->cmd_.wait.init_time > 50;
        };
        b->cmd_.wait.exit_to_mode = M::SetPhis_Stop;
      }
    } break;
    case M::SetPhis_Stop: {
      Serial.println("ModeRunners::SetPhis(Stop)");

      b->CommandBoth([](Servo* s) { s->SetStop(); });
      m = c.exit_to_mode;
    } break;
    default:
      break;
  }
}
