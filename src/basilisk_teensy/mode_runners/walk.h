#pragma once

#include "meta.h"

void ModeRunners::Walk(Basilisk* b) {
  auto& m = b->cmd_.mode;
  auto& c = b->cmd_.walk;
  auto& ps = b->cmd_.pivseq;

  switch (m) {
    case M::Walk: {
      m = M::PivSeq_Init;
      ps.pivots = [](Basilisk* b, int idx) {
        auto& c = b->cmd_.walk;
        Basilisk::Command::Pivot p;
        p.didimbal = idx % 2 == 0 ? c.init_didimbal : !c.init_didimbal;
        const uint8_t didim_idx = p.didimbal == BOOL_L ? IDX_L : IDX_R;
        const uint8_t kick_idx = p.didimbal == BOOL_L ? IDX_R : IDX_L;
        p.tgt_yaw = c.tgt_yaw[didim_idx];
        p.stride = c.stride[didim_idx](b);
        if (idx == 0) {
          p.bend[didim_idx] = c.bend[didim_idx];
        } else {
          p.bend[didim_idx] = NaN;
        }
        p.bend[kick_idx] = c.bend[kick_idx];
        p.speed = c.speed[didim_idx];
        p.acclim = c.acclim[didim_idx];
        p.min_dur = c.min_stepdur[didim_idx];
        p.max_dur = c.max_stepdur[didim_idx];
        p.exit_condition = c.exit_condition;
        return p;
      };
      ps.intervals = [](Basilisk* b, int idx) {
        auto& c = b->cmd_.walk;
        const LR didimbal = idx % 2 == 0 ? c.init_didimbal : !c.init_didimbal;
        const uint8_t didim_idx = didimbal == BOOL_L ? IDX_L : IDX_R;
        return c.interval[didim_idx];
      };
      ps.steps = c.steps;
      ps.exit_condition = c.exit_condition;
      ps.exit_to_mode = M::Idle_Init;
    } break;
    default:
      break;
  }
}
