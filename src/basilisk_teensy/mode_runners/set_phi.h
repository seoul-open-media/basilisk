#pragma once

#include "mode_runners.h"

void ModeRunners::SetPhi(Basilisk* b) {
  auto& m = b->cmd_.mode;

  switch (m) {
    case M::SetPhi: {
      auto& cmd = b->cmd_.set_phi;

      bool exit[2] = {false, false};

      for (uint8_t i = 0; i < 2; i++) {
        auto* s = b->lr_[i];
        const auto& tgt_outpos = cmd.tgt_phi[i];
        auto tgt_outvel = cmd.tgt_phidot[i];
        const auto& tgt_outacc = cmd.tgt_phiddot[i];

        double tgt_rtrvel;
        double tgt_rtracc;

        if (isnan(tgt_outpos)) {
          tgt_rtrvel = b->gear_rat_ * tgt_outvel;
          tgt_rtracc = b->gear_rat_ * tgt_outacc;
        } else {
          const auto cur_outpos = s->GetReply().abs_position;
          const auto tgt_delta_outpos = tgt_outpos - cur_outpos;

          if (abs(tgt_delta_outpos) >= cmd.fix_thr) {
            tgt_outvel *= constrain(tgt_delta_outpos / cmd.damp_thr, -1.0, 1.0);
            tgt_rtrvel = b->gear_rat_ * tgt_outvel;
            tgt_rtracc = b->gear_rat_ * tgt_outacc;
          } else {
            tgt_rtrvel = 0.0;
            tgt_rtracc = 0.0;
            exit[i] = true;
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
        m = b->cmd_.set_phi.exit_to_mode;
      }
    } break;
    default:
      break;
  }
}
