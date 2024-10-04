#pragma once

#include "meta.h"

void Presets::RMagRls(Basilisk* b) {
  b->cmd_.mode = M::Idle_Nop;
  b->mags_.SetStrength(2, MagStren::Min);
  b->mags_.SetStrength(3, MagStren::Min);
}

void Presets::RMagAtt(Basilisk* b) {
  b->cmd_.mode = M::Idle_Nop;
  b->mags_.SetStrength(2, MagStren::Max);
  b->mags_.SetStrength(3, MagStren::Max);
}

void Presets::LMagRls(Basilisk* b) {
  b->cmd_.mode = M::Idle_Nop;
  b->mags_.SetStrength(0, MagStren::Min);
  b->mags_.SetStrength(1, MagStren::Min);
}

void Presets::LMagAtt(Basilisk* b) {
  b->cmd_.mode = M::Idle_Nop;
  b->mags_.SetStrength(0, MagStren::Max);
  b->mags_.SetStrength(1, MagStren::Max);
}

void Presets::RandomMagsWeak(Basilisk* b) {
  b->cmd_.mode = M::RandomMags_Init;
  b->cmd_.random_mags.min_phase_dur = 1000;
  b->cmd_.random_mags.max_phase_dur = 3000;
  b->cmd_.random_mags.dur = 5000;
}

void Presets::RandomMagsStrong(Basilisk* b) {
  b->cmd_.mode = M::RandomMags_Init;
  b->cmd_.random_mags.min_phase_dur = 10;
  b->cmd_.random_mags.max_phase_dur = 100;
  b->cmd_.random_mags.dur = 5000;
}
