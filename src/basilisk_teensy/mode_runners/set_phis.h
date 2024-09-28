#pragma once

#include "meta.h"

void ModeRunners::SetPhis(Basilisk* b) {
  static uint32_t init_time;
  static uint8_t fixing_cycles[2];

  auto& m = b->cmd_.mode;
  auto& c = b->cmd_.set_phis;

  switch (m) {
    case M::SetPhis_Init: {
      init_time = millis();
      for (uint8_t f : IDX_LR) fixing_cycles[f] = 0;
      m = M::SetPhis_Move;
    } break;
    case M::SetPhis_Move: {
      if ([&] {
            if (millis() - init_time > c.max_dur ||
                (c.exit_condition && c.exit_condition(b))) {
              return true;
            }

            for (const uint8_t f : IDX_LR) {
              auto* s = b->s_[f];

              double tgt_rtrvel;
              double tgt_rtracclim;

              if (c.tgt_phi[f].isnan()) {
                // Velocity control with no target position is NOT allowed
                // for Basilisk.
                tgt_rtrvel = 0.0;
                tgt_rtracclim = 32.0;
                pp(fixing_cycles[f]);
              } else {
                const auto cur_outpos = s->GetReply().abs_position;
                const auto tgt_delta_outpos = c.tgt_phi[f] - cur_outpos;
                if (abs(tgt_delta_outpos) < c.fix_thr) {
                  // Target phi is reached.
                  tgt_rtrvel = 0.0;
                  tgt_rtracclim = 32.0;
                  pp(fixing_cycles[f]);
                } else {
                  tgt_rtrvel =
                      b->gr_ * c.tgt_phispeed[f] *
                      /* dx/dt = -x^p where x(t=0) = 1 gives:
                       * x = e^{-t} for p = 1
                       * x = (1 - (1 - p)t)^{1 / (1 - p)} elsewhere
                       * x converges to 0 asymptotically if p >= 1,
                       *   rate of convergence decreasing as p increases.
                       * On the other hand, x hits 0 in finite time if p < 1,
                       *   rate of which increases as p decreases.
                       * Therefore, damping is achieved if p > 0. */
                      signedpow(
                          constrain(tgt_delta_outpos / c.damp_thr, -1.0, 1.0),
                          0.6);
                  tgt_rtracclim = b->gr_ * c.tgt_phiacclim[f];
                  fixing_cycles[f] = 0;
                }
              }
#if I_WANT_DEBUG
              Serial.print(f == IDX_L ? "l_" : "r_");
              Serial.print("c.tgt_phi[f] ");
              Serial.print(c.tgt_phi[f]);
              Serial.print(" cur_outpos ");
              Serial.print(s->GetReply().abs_position);
              Serial.print(" tgt_rtrvel ");
              Serial.print(tgt_rtrvel);
              Serial.print(" tgt_rtracclim ");
              Serial.print(tgt_rtracclim);
              Serial.println();
#endif
              s->SetPosition([&] {
                auto pm_cmd = *b->pm_cmd_template_;
                pm_cmd.position = NaN;
                pm_cmd.velocity = tgt_rtrvel;
                pm_cmd.accel_limit = tgt_rtracclim;
                return pm_cmd;
              }());
            }

            return fixing_cycles[IDX_L] >= c.fixing_cycles_thr &&
                   fixing_cycles[IDX_R] >= c.fixing_cycles_thr &&
                   (millis() - init_time > c.min_dur);
          }()) {
        m = c.exit_to_mode;
      }
    } break;
    default:
      break;
  }
}
