#pragma once

#include "mode_runners.h"

void ModeRunners::SetPhis(Basilisk* b) {
  static uint32_t init_time;
  static bool stop_this_cycle[2];
  static bool stopped[2];
  static bool fixing[2];

  auto& m = b->cmd_.mode;
  auto& c = b->cmd_.set_phis;

  switch (m) {
    case M::SetPhis_Init: {
      Serial.println("ModeRunners::SetPhis(Init)");

      for (const uint8_t f : IDX_LR) {
        stop_this_cycle[f] = false;
        stopped[f] = false;
        fixing[f] = false;
      }
      init_time = millis();
      m = M::SetPhis_Move;
    } break;
    case M::SetPhis_Move: {
      // Serial.println("ModeRunners::SetPhis(Move)");

      if ([&] {
            if (millis() - init_time > c.max_dur ||
                (c.exit_condition && c.exit_condition(b))) {
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
                fixing[f] = true;
              } else if (!stopped[f]) {
                const auto cur_outpos = s->GetReply().abs_position;
                const auto tgt_delta_outpos = tgt_outpos - cur_outpos;
                if (abs(tgt_delta_outpos) < c.stop_thr) {
                  // Target phi is reached.
                  stop_this_cycle[f] = true;
                } else {
                  const double tgt_outvel =
                      c.tgt_phispeed[f] *
                      signedpow(
                          constrain(tgt_delta_outpos / c.damp_thr, -1.0, 1.0),
                          0.85);
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

                  stop_this_cycle[f] = false;
                  stopped[f] = false;
                }
              }

              if (fixing[f]) {
                s->SetPosition([&] {
                  auto pm_cmd = *b->pm_cmd_template_;
                  pm_cmd.position = NaN;
                  pm_cmd.velocity = 0.0;
                  pm_cmd.accel_limit = 0.0;
                  return pm_cmd;
                }());
              } else {
                if (stop_this_cycle[f]) {
                  s->SetStop();
                  stopped[f] = true;
                  stop_this_cycle[f] = false;
                } else if (!stopped[f]) {
                  s->SetPosition([&] {
                    auto pm_cmd = *b->pm_cmd_template_;
                    pm_cmd.position = NaN;
                    pm_cmd.velocity = tgt_rtrvel;
                    pm_cmd.accel_limit = tgt_rtracclim;
                    return pm_cmd;
                  }());
                }
              }
            }

            return (fixing[IDX_L] || stopped[IDX_L]) &&
                   (fixing[IDX_R] || stopped[IDX_R]) &&
                   (millis() - init_time > c.min_dur);
          }()) {
        m = c.exit_to_mode;
      }
    } break;
    default:
      break;
  }
}
