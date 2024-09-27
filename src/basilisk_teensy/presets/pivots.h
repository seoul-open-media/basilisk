#pragma once

#include "meta.h"

#define YAW_TO_AUD (-0.25)

void Presets::PivRFr45(Basilisk* b) {
  b->cmd_.mode = M::Pivot_Init;
  auto& c = b->cmd_.pivot;
  c.didimbal = BOOL_L;
  c.tgt_yaw = [](Basilisk*) { return YAW_TO_AUD; };
  c.stride = 0.125;
  c.bend[IDX_L] = NaN;
  c.bend[IDX_R] = 0.0;
  c.speed = 0.1;
  c.acclim = 1.0;
  c.min_dur = 0;
  c.max_dur = 3000;
  c.exit_condition = nullptr;
}

void Presets::PivLFr45(Basilisk* b) {
  b->cmd_.mode = M::Pivot_Init;
  auto& c = b->cmd_.pivot;
  c.didimbal = BOOL_R;
  c.tgt_yaw = [](Basilisk*) { return YAW_TO_AUD; };
  c.stride = 0.125;
  c.bend[IDX_L] = 0.0;
  c.bend[IDX_R] = NaN;
  c.speed = 0.1;
  c.acclim = 1.0;
  c.min_dur = 0;
  c.max_dur = 3000;
  c.exit_condition = nullptr;
}

void Presets::PivRBk45(Basilisk* b) {
  b->cmd_.mode = M::Pivot_Init;
  auto& c = b->cmd_.pivot;
  c.didimbal = BOOL_L;
  c.tgt_yaw = [](Basilisk*) { return YAW_TO_AUD; };
  c.stride = -0.125;
  c.bend[IDX_L] = NaN;
  c.bend[IDX_R] = 0.0;
  c.speed = 0.1;
  c.acclim = 1.0;
  c.min_dur = 0;
  c.max_dur = 3000;
  c.exit_condition = nullptr;
}

void Presets::PivLBk45(Basilisk* b) {
  b->cmd_.mode = M::Pivot_Init;
  auto& c = b->cmd_.pivot;
  c.didimbal = BOOL_R;
  c.tgt_yaw = [](Basilisk*) { return YAW_TO_AUD; };
  c.stride = -0.125;
  c.bend[IDX_L] = 0.0;
  c.bend[IDX_R] = NaN;
  c.speed = 0.1;
  c.acclim = 1.0;
  c.min_dur = 0;
  c.max_dur = 3000;
  c.exit_condition = nullptr;
}

void Presets::PivRFr90(Basilisk* b) {
  b->cmd_.mode = M::Pivot_Init;
  auto& c = b->cmd_.pivot;
  c.didimbal = BOOL_L;
  c.tgt_yaw = [](Basilisk*) { return YAW_TO_AUD; };
  c.stride = 0.25;
  c.bend[IDX_L] = NaN;
  c.bend[IDX_R] = 0.0;
  c.speed = 0.2;
  c.acclim = 1.0;
  c.min_dur = 0;
  c.max_dur = 5000;
  c.exit_condition = nullptr;
}

void Presets::PivLFr90(Basilisk* b) {
  b->cmd_.mode = M::Pivot_Init;
  auto& c = b->cmd_.pivot;
  c.didimbal = BOOL_R;
  c.tgt_yaw = [](Basilisk*) { return YAW_TO_AUD; };
  c.stride = 0.25;
  c.bend[IDX_L] = 0.0;
  c.bend[IDX_R] = NaN;
  c.speed = 0.2;
  c.acclim = 1.0;
  c.min_dur = 0;
  c.max_dur = 5000;
  c.exit_condition = nullptr;
}

void Presets::PivRBk90(Basilisk* b) {
  b->cmd_.mode = M::Pivot_Init;
  auto& c = b->cmd_.pivot;
  c.didimbal = BOOL_L;
  c.tgt_yaw = [](Basilisk*) { return YAW_TO_AUD; };
  c.stride = -0.25;
  c.bend[IDX_L] = NaN;
  c.bend[IDX_R] = 0.0;
  c.speed = 0.2;
  c.acclim = 1.0;
  c.min_dur = 0;
  c.max_dur = 5000;
  c.exit_condition = nullptr;
}

void Presets::PivLBk90(Basilisk* b) {
  b->cmd_.mode = M::Pivot_Init;
  auto& c = b->cmd_.pivot;
  c.didimbal = BOOL_R;
  c.tgt_yaw = [](Basilisk*) { return YAW_TO_AUD; };
  c.stride = -0.25;
  c.bend[IDX_L] = 0.0;
  c.bend[IDX_R] = NaN;
  c.speed = 0.2;
  c.acclim = 1.0;
  c.min_dur = 0;
  c.max_dur = 5000;
  c.exit_condition = nullptr;
}

void Presets::BendRIn45(Basilisk* b) {
  b->cmd_.mode = M::Pivot_Init;
  auto& c = b->cmd_.pivot;
  c.didimbal = BOOL_R;
  c.tgt_yaw = [](Basilisk*) { return NaN; };
  c.stride = NaN;
  c.bend[IDX_L] = 0.0;  // Ignored anyway.
  c.bend[IDX_R] = 0.125;
  c.speed = 0.15;
  c.acclim = 1.0;
  c.min_dur = 0;
  c.max_dur = 5000;
  c.exit_condition = nullptr;
}

void Presets::BendROut45(Basilisk* b) {
  b->cmd_.mode = M::Pivot_Init;
  auto& c = b->cmd_.pivot;
  c.didimbal = BOOL_R;
  c.tgt_yaw = [](Basilisk*) { return NaN; };
  c.stride = NaN;
  c.bend[IDX_L] = 0.0;  // Ignored anyway.
  c.bend[IDX_R] = -0.125;
  c.speed = 0.15;
  c.acclim = 1.0;
  c.min_dur = 0;
  c.max_dur = 5000;
  c.exit_condition = nullptr;
}
