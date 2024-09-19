#pragma once

#include "mode_runners.h"

void ModeRunners::SetPhis(Basilisk* b) {
  auto& m = b->cmd_.mode;
  auto& c = b->cmd_.set_phis;

  switch (m) {
    case M::SetPhis_Init: {
      // Serial.println("ModeRunners::SetPhis(Init)");
      for (const uint8_t f : IDX_LR) c.fix_cycles[f] = 0;
      c.init_time = millis();
      m = M::SetPhis_Move;
    } break;
    case M::SetPhis_Move: {
      // Serial.println("ModeRunners::SetPhis(Move)");
      if ([&] {
            if (millis() - c.init_time > c.max_dur) {
              return true;
            }

            for (const uint8_t f : IDX_LR) {
              auto* s = b->s_[f];

              const double tgt_outpos = c.tgt_phi[f];

              double tgt_rtrvel;
              double tgt_rtracclim;

              if (isnan(tgt_outpos)) {
                // Velocity control with no target position is NOT allowed
                // for Basilisk.
                tgt_rtrvel = 0.0;
                tgt_rtracclim = 0.0;
                pp(c.fix_cycles[f]);
              } else {
                const auto cur_outpos = s->GetReply().abs_position;
                const auto tgt_delta_outpos = tgt_outpos - cur_outpos;
                if (abs(tgt_delta_outpos) < c.fix_thr) {
                  // Target phi is reached.
                  tgt_rtrvel = 0.0;
                  tgt_rtracclim = 0.0;
                  pp(c.fix_cycles[f]);
                } else {
                  const double tgt_outvel =
                      c.tgt_phispeed[f] *
                      signedpow(
                          constrain(tgt_delta_outpos / c.damp_thr, -1.0, 1.0),
                          0.875);
                  /* dx/dt = -x^p where x(t=0) = 1 gives:
                   * x = e^{-t} for p = 1
                   * x = (1 - (1 - p)t)^{1 / (1 - p)} elsewhere
                   * x converges to 0 asymptotically if p >= 1,
                   *   rate of convergence decreasing as p increases.
                   * On the other hand, x hits 0 in finite time if p < 1,
                   *   rate of which increases as p decreases.
                   * Therefore, damping is achieved if p > 0. */
                  tgt_rtrvel = b->gr_ * tgt_outvel;

                  const double tgt_outacclim = c.tgt_phiacclim[f];
                  tgt_rtracclim = b->gr_ * tgt_outacclim;

                  c.fix_cycles[f] = 0;
                }
              }

              s->SetPosition([&] {
                auto pm_cmd = *b->pm_cmd_template_;
                pm_cmd.position = NaN;
                pm_cmd.velocity = tgt_rtrvel;
                pm_cmd.accel_limit = tgt_rtracclim;
                return pm_cmd;
              }());
            }

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
