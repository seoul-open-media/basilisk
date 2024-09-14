#pragma once

#include "mode_runners.h"

void ModeRunners::SetPhi(Basilisk* b) {
  auto& m = b->cmd_.mode;
  auto& c = b->cmd_.set_phi;

  switch (m) {
    case M::SetPhi_Init: {
      // Serial.println("ModeRunners::SetPhi(Init)");

      bool exit[2];

      for (uint8_t i = 0; i < 2; i++) {
        auto* s = b->lr_[i];

        const auto& tgt_outpos = c.tgt_phi[i];
        auto tgt_outvel = c.tgt_phidot[i];
        const auto& tgt_outacc = c.tgt_phiddot[i];

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
            tgt_outvel *= constrain(tgt_delta_outpos / c.damp_thr, -1.0, 1.0);
            tgt_rtrvel = b->gear_rat_ * tgt_outvel;
            tgt_rtracc = b->gear_rat_ * tgt_outacc;
            exit[i] = false;
          }
        }

        s->SetPosition([&] {
          auto pm_cmd = *b->pm_cmd_template_;
          pm_cmd.position = NaN;
          pm_cmd.velocity = tgt_rtrvel;
          pm_cmd.accel_limit = tgt_rtracc;

          // Serial.print(i == 0 ? "l_" : "r_");
          // Serial.print("tgt_outpos:");
          // Serial.println(tgt_outpos);
          // Serial.print(i == 0 ? "l_" : "r_");
          // Serial.print("cur_outpos:");
          // Serial.println(s->GetReply().abs_position);
          // Serial.print(i == 0 ? "l_" : "r_");
          // Serial.print("pm_cmd.velocity:");
          // Serial.println(pm_cmd.velocity);

          return pm_cmd;
        }());
      }

      if (exit[0] && exit[1]) {
        m = M::Wait;
        b->cmd_.wait.init_time = millis();
        b->cmd_.wait.exit_condition = [](Basilisk* b) {
          return millis() - b->cmd_.wait.init_time > 50;
        };
        b->cmd_.wait.exit_to_mode = M::SetPhi_Stop;
      }
    } break;
    case M::SetPhi_Stop: {
      Serial.println("ModeRunners::SetPhi(Stop)");

      b->CommandBoth([](Servo* s) { s->SetStop(); });
      m = c.exit_to_mode;
    } break;
    default:
      break;
  }
}
