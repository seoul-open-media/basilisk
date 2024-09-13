#pragma once

#include "mode_runners.h"

void ModeRunners::SetPhi(Basilisk* b) {
  auto& m = b->cmd_.mode;
  auto& c = b->cmd_.set_phi;

  switch (m) {
    case M::SetPhi: {
      Serial.println("ModeRunners::SetPhi");

      bool exit[2] = {true, true};

      for (uint8_t i = 0; i < 2; i++) {
        auto* s = b->lr_[i];
        const auto& tgt_outpos = c.tgt_phi[i];
        auto tgt_outvel = c.tgt_phidot[i];
        const auto& tgt_outacc = c.tgt_phiddot[i];

        double tgt_rtrvel;
        double tgt_rtracc;

        if (isnan(tgt_outpos)) {
          tgt_rtrvel = b->gear_rat_ * tgt_outvel;
          tgt_rtracc = b->gear_rat_ * tgt_outacc;
        } else {
          const auto cur_outpos = s->GetReply().abs_position;
          const auto tgt_delta_outpos = tgt_outpos - cur_outpos;

          if (abs(tgt_delta_outpos) >= c.fix_thr) {
            tgt_outvel *= constrain(tgt_delta_outpos / c.damp_thr, -1.0, 1.0);
            tgt_rtrvel = b->gear_rat_ * tgt_outvel;
            tgt_rtracc = b->gear_rat_ * tgt_outacc;
            exit[i] = false;
          } else {
            tgt_rtrvel = 0.0;
            tgt_rtracc = 0.0;
          }
        }

        s->MakePosition([&] {
          auto pm_cmd = *b->pm_cmd_template_;
          pm_cmd.position = NaN;
          pm_cmd.velocity = tgt_rtrvel;
          pm_cmd.accel_limit = tgt_rtracc;
          return pm_cmd;
        }());
      }

      if (exit[0] && exit[1]) {
        m = c.exit_to_mode;
      }
    } break;
    default:
      break;
  }
}
