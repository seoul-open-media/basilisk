#pragma once

#include "../helpers/imports.h"
#include "../servo_units/basilisk.h"

namespace tests {
using M = Basilisk::Command::Mode;

void Pivot(Basilisk* b) {
  auto& m = b->cmd_.mode;
  auto& c = b->cmd_.pivot;

  m = M::Pivot_Init;
  c.didimbal = BOOL_L;
  c.tgt_yaw = [](Basilisk*) { return NaN; };
  c.stride = 0.125;
  c.bend[IDX_L] = 0.0;
  c.bend[IDX_R] = -0.125;
  c.speed = globals::stdval::speed::normal;
  c.acclim = globals::stdval::acclim::normal;
  c.min_dur = 2000;
  c.max_dur = -1;
  c.exit_to_mode = M::Idle_Init;
}

void PivSpin(Basilisk* b) {
  auto& m = b->cmd_.mode;
  auto& c = b->cmd_.piv_spin;

  m = M::PivSpin;
  c.didimbal = BOOL_L;
  c.dest_yaw = NaN;
  c.exit_thr = NaN;
  c.stride = 0.125;
  c.bend[IDX_L] = 0.0;
  c.bend[IDX_R] = 0.0;
  c.speed = globals::stdval::speed::normal;
  c.acclim = globals::stdval::acclim::normal;
  c.min_stepdur = 0;
  c.max_stepdur = -1;
  c.interval = 0;
  c.steps = -1;
}

void WalkToDir(Basilisk* b) {
  auto& m = b->cmd_.mode;
  auto& c = b->cmd_.walk_to_dir;

  m = M::WalkToDir;
  c.init_didimbal = BOOL_L;
  c.tgt_yaw = 0.0;
  c.stride = 0.125;
  c.bend[IDX_L] = 0.0;
  c.bend[IDX_R] = 0.0;
  c.speed = globals::stdval::speed::normal;
  c.acclim = globals::stdval::acclim::normal;
  c.min_stepdur = 1000;
  c.max_stepdur = 3000;
  c.interval = 0;
  c.steps = -1;
}

void WalkToPos(Basilisk* b) {
  auto& m = b->cmd_.mode;
  auto& c = b->cmd_.walk_to_pos;

  m = M::WalkToPos;
  c.init_didimbal = BOOL_L;
  c.tgt_pos = Vec2{(b->cfg_.lps.minx + b->cfg_.lps.maxx) * 0.5,
                   (b->cfg_.lps.miny + b->cfg_.lps.maxy) * 0.5};
  c.dist_thr = 30;
  c.stride = 0.125;
  c.bend[IDX_L] = 0.0;
  c.bend[IDX_R] = 0.0;
  c.speed = globals::stdval::speed::normal;
  c.acclim = globals::stdval::acclim::normal;
  c.min_stepdur = 1000;
  c.max_stepdur = 3000;
  c.interval = 0;
  c.steps = -1;
}

void Sufi(Basilisk* b) {
  auto& m = b->cmd_.mode;
  auto& c = b->cmd_.sufi;

  c.init_didimbal = BOOL_L;
  c.dest_yaw = NaN;
  c.exit_thr = NaN;
  c.stride = 0.125;
  c.bend[IDX_L] = 0.0;
  c.bend[IDX_R] = 0.0;
  c.speed = globals::stdval::speed::normal;
  c.acclim = globals::stdval::acclim::normal;
  c.min_stepdur = 1000;
  c.max_stepdur = 3000;
  c.interval = 0;
  c.steps = -1;
}

}  // namespace tests
