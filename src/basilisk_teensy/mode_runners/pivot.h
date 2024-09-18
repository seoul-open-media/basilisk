#pragma once

#include "mode_runners.h"

void ModeRunners::Pivot(Basilisk* b) {
  auto& m = b->cmd_.mode;
  auto& c = b->cmd_.pivot;
  static double init_yaw;
  static double interm_yaw;
  static double tgt_yaw;

  const uint8_t didim_idx = c.didimbal == BOOL_L ? IDX_L : IDX_R;
  const uint8_t kick_idx = c.didimbal == BOOL_L ? IDX_R : IDX_L;
  auto& mags = b->cmd_.set_mags;
  auto& phis = b->cmd_.set_phis;

  switch (m) {
    case M::Pivot_Init: {
      // Serial.println("ModeRunners::Pivot(Init)");

      c.init_time = millis();

      init_yaw = b->imu_.GetYaw(true);
      tgt_yaw = c.tgt_yaw(b);

      // Check if we need to set didimbal.
      if (c.bend[didim_idx].isnan()) {
        m = M::Pivot_Kick;
        return;
      }

      // Release didimbal, attach kickbal.
      m = M::SetMags_Init;
      const bool attach_l = c.didimbal != BOOL_L;
      const bool attach_r = c.didimbal != BOOL_R;
      mags.strengths[0] = Bool2MS(attach_l);
      mags.strengths[1] = Bool2MS(attach_l);
      mags.strengths[2] = Bool2MS(attach_r);
      mags.strengths[3] = Bool2MS(attach_r);
      mags.expected_state[IDX_L] = attach_l;
      mags.expected_state[IDX_R] = attach_r;
      mags.verif_thr = 5;
      mags.min_dur = 50;
      mags.max_dur = 200;
      mags.exit_to_mode = M::SetPhis_Init;
      phis.tgt_phi[didim_idx] =
          b->imu_.GetYaw(true) - tgt_yaw - c.bend[didim_idx];
      phis.tgt_phispeed[didim_idx] = c.speed;
      phis.tgt_phiacclim[didim_idx] = c.acclim;
      phis.tgt_phi[kick_idx] = NaN;
      phis.tgt_phispeed[kick_idx] = 0.0;
      phis.tgt_phiacclim[kick_idx] = 0.0;
      phis.damp_thr = 0.025;
      phis.fix_thr = 0.01;
      phis.fix_cycles_thr = 5;
      phis.min_dur = 0;
      phis.max_dur = c.max_dur / 4;
      phis.exit_to_mode = M::Pivot_Kick;
    } break;
    case M::Pivot_Kick: {
      // Serial.println("ModeRunners::Pivot(Kick)");

      m = M::SetMags_Init;
      const bool attach_l = c.didimbal == BOOL_L;
      const bool attach_r = c.didimbal == BOOL_R;
      mags.strengths[0] = Bool2MS(attach_l);
      mags.strengths[1] = Bool2MS(attach_l);
      mags.strengths[2] = Bool2MS(attach_r);
      mags.strengths[3] = Bool2MS(attach_r);
      mags.expected_state[IDX_L] = attach_l;
      mags.expected_state[IDX_R] = attach_r;
      mags.verif_thr = 5;
      mags.min_dur = 50;
      mags.max_dur = 200;
      mags.exit_to_mode = M::SetPhis_Init;
      const auto sgnd_stride = c.stride * (c.didimbal == BOOL_L ? 1.0 : -1.0);
      interm_yaw = b->imu_.GetYaw(true);
      const auto interm_error = interm_yaw - init_yaw;
      phis.tgt_phi[didim_idx] = sgnd_stride - c.bend[didim_idx] - interm_error;
      phis.tgt_phi[kick_idx] = -c.bend[kick_idx] + sgnd_stride;
      phis.tgt_phispeed[didim_idx] = c.speed;
      phis.tgt_phispeed[kick_idx] = c.speed;
      phis.tgt_phiacclim[didim_idx] = c.acclim;
      phis.tgt_phiacclim[kick_idx] = c.acclim;
      phis.damp_thr = 0.025;
      phis.fix_thr = 0.01;
      phis.fix_cycles_thr = 5;
      phis.min_dur = c.min_dur > (millis() - c.init_time)
                         ? c.min_dur - (millis() - c.init_time)
                         : 0;
      phis.max_dur = c.max_dur > (millis() - c.init_time)
                         ? c.max_dur - (millis() - c.init_time)
                         : 0;
      phis.exit_to_mode = c.exit_to_mode;
    } break;
    default:
      break;
  }
}
