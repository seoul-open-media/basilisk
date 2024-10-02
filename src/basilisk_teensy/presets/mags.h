#pragma once

#include "meta.h"

void Presets::RMagRls(Basilisk* b) {
  b->cmd_.mode = M::Idle_Nop;
  b->mags_.SetStrength(2, MagnetStrength::Min);
  b->mags_.SetStrength(3, MagnetStrength::Min);
}

void Presets::RMagAtt(Basilisk* b) {
  b->cmd_.mode = M::Idle_Nop;
  b->mags_.SetStrength(2, MagnetStrength::Max);
  b->mags_.SetStrength(3, MagnetStrength::Max);
}

void Presets::LMagRls(Basilisk* b) {
  b->cmd_.mode = M::Idle_Nop;
  b->mags_.SetStrength(0, MagnetStrength::Min);
  b->mags_.SetStrength(1, MagnetStrength::Min);
}

void Presets::LMagAtt(Basilisk* b) {
  b->cmd_.mode = M::Idle_Nop;
  b->mags_.SetStrength(0, MagnetStrength::Max);
  b->mags_.SetStrength(1, MagnetStrength::Max);
}

void Presets::RandomMagsWeak(Basilisk* b) {
  b->cmd_.mode = M::RandomMags;
  b->cmd_.random_mags.min_phase_dur = 1000;
  b->cmd_.random_mags.max_phase_dur = 3000;
}

void Presets::RandomMagsStrong(Basilisk* b) {
  b->cmd_.mode = M::RandomMags;
  b->cmd_.random_mags.min_phase_dur = 10;
  b->cmd_.random_mags.max_phase_dur = 100;
}
