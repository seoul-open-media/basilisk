#pragma once

#include "mode_runners.h"

void ModeRunners::Walk(Basilisk* b) {
  auto& m = b->cmd_.mode;
  auto& c = b->cmd_.walk;
  auto& ps = b->cmd_.pivseq;

  switch (m) {
    case M::Walk: {
      // Serial.println("ModeRunners::Walk");

      m = M::PivSeq_Init;
      ps.pivots = [](Basilisk* b, int idx) {
        auto& c = b->cmd_.walk;
        Basilisk::Command::Pivot p;
        p.didimbal = idx % 2 == 0 ? c.init_didimbal : !c.init_didimbal;
        uint8_t didim_idx = p.didimbal == BOOL_L ? IDX_L : IDX_R;
        p.tgt_yaw = c.tgt_yaw[didim_idx];
        p.stride = c.stride[didim_idx](b);
        p.speed = c.speed[didim_idx];
        p.acclim = c.acclim[didim_idx];
        p.min_dur = c.min_stepdur[didim_idx];
        p.max_dur = c.max_stepdur[didim_idx];
        p.bend[IDX_L] = c.bend[IDX_L];
        p.bend[IDX_R] = c.bend[IDX_R];
        p.exit_condition = c.exit_condition;
        return p;
      };
      ps.intervals = [](Basilisk* b, int idx) {
        auto& c = b->cmd_.walk;
        LR didimbal = idx % 2 == 0 ? c.init_didimbal : !c.init_didimbal;
        uint8_t didim_idx = didimbal == BOOL_L ? IDX_L : IDX_R;
        return c.interval[didim_idx];
      };
      ps.loop_begin_idx = 0;
      ps.loop_end_idx = 2;
      ps.steps = c.steps;
      ps.exit_condition = c.exit_condition;
      ps.exit_to_mode = M::Idle_Init;
    } break;
    default:
      break;
  }
}
