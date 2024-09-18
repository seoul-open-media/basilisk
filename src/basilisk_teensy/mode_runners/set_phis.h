#pragma once

#include "mode_runners.h"

void ModeRunners::SetPhis(Basilisk* b) {
  auto& m = b->cmd_.mode;
  auto& c = b->cmd_.set_phis;

  switch (m) {
    case M::SetPhis_Init: {
      Serial.println("ModeRunners::SetPhis(Init)");

      for (const uint8_t f : IDX_LR) c.fix_cycles[f] = 0;
      c.init_time = millis();
      m = M::SetPhis_Move;
    } break;
    case M::SetPhis_Move: {
      Serial.println("ModeRunners::SetPhis(Move)");

      if ([&] {
            if (millis() - c.init_time > c.max_dur) {
              return true;
            }

            for (const uint8_t f : IDX_LR) {
              auto* s = b->lr_[f];

              const double tgt_outpos = c.tgt_phi[f];

              double tgt_rtrvel;
              double tgt_rtracc;

              if (isnan(tgt_outpos)) {
                // Velocity control with no target position is NOT allowed
                // for Basilisk.
                tgt_rtrvel = 0.0;
                tgt_rtracc = 0.0;
                pp(c.fix_cycles[f]);

                Serial.println("Fix++");
              } else {
                const auto cur_outpos = s->GetReply().abs_position;
                const auto tgt_delta_outpos = tgt_outpos - cur_outpos;
                if (abs(tgt_delta_outpos) < c.fix_thr) {
                  // Target phi is reached.
                  tgt_rtrvel = 0.0;
                  tgt_rtracc = 0.0;
                  pp(c.fix_cycles[f]);

                  Serial.println("Fix++");
                } else {
                  const double tgt_outvel =
                      c.tgt_phispeed[f] *
                      signedpow(
                          constrain(tgt_delta_outpos / c.damp_thr, -1.0, 1.0),
                          1.5);
                  tgt_rtrvel = b->gr_ * tgt_outvel;

                  const double tgt_outacc = c.tgt_phiacc[f];
                  tgt_rtracc = b->gr_ * tgt_outacc;

                  c.fix_cycles[f] = 0;

                  Serial.print("tgt_outpos:");
                  Serial.print(tgt_outpos);
                  Serial.print(",");
                  Serial.print("cur_outpos:");
                  Serial.print(cur_outpos);
                  Serial.print(",");
                  Serial.print("tgt_delta_outpos:");
                  Serial.print(tgt_delta_outpos);
                  Serial.print(",");
                  Serial.print("tgt_outvel:");
                  Serial.print(tgt_outvel);
                  Serial.print(",");
                  Serial.print("tgt_rtrvel:");
                  Serial.print(tgt_rtrvel);
                  Serial.println();
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

            // Serial.print("fix_cycles[0]:");
            // Serial.print(c.fix_cycles[0]);
            // Serial.print(",");
            // Serial.print("fix_cycles[1]:");
            // Serial.print(c.fix_cycles[1]);
            // Serial.print(",");
            // Serial.print("fix_cycles_thr:");
            // Serial.print(c.fix_cycles_thr);
            // Serial.print(",");
            // Serial.print("millis() - c.init_time:");
            // Serial.print(millis() - c.init_time);
            // Serial.print(",");
            // Serial.print("min_dur:");
            // Serial.print(c.min_dur);
            // Serial.println();

            return (c.fix_cycles[0] >= c.fix_cycles_thr) &&
                   (c.fix_cycles[1] >= c.fix_cycles_thr) &&
                   (millis() - c.init_time > c.min_dur);
          }()) {
        m = c.exit_to_mode;
      }
    } break;
    default:
      break;
  }
}
