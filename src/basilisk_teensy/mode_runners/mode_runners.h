#pragma once

#include <map>

#include "../servo_units/basilisk.h"

struct ModeRunners {
  using M = Basilisk::Command::Mode;

  static void Idle(Basilisk*);
  static void Wait(Basilisk*);
  static void Free(Basilisk*);
  static void SetMags(Basilisk*);
  static void SetPhis(Basilisk*);
  static void Pivot(Basilisk*);
  static void PivSeq(Basilisk*);
  static void WalkToDir(Basilisk*);
  static void WalkToPos(Basilisk*);
  static void Sufi(Basilisk*);

  inline static const std::map<M, void (*)(Basilisk*)> mode_runners = {
      {M::Idle_Init, &Idle},
      {M::Idle_Nop, &Idle},
      {M::Wait, &Wait},
      {M::Free, &Free},
      {M::SetMags_Init, &SetMags},
      {M::SetMags_Wait, &SetMags},
      {M::SetPhis_Init, &SetPhis},
      {M::SetPhis_Move, &SetPhis},
      {M::Pivot_Init, &Pivot},
      {M::Pivot_Kick, &Pivot},
      {M::PivSeq_Init, &PivSeq},
      {M::PivSeq_Step, &PivSeq},
      {M::WalkToDir, &WalkToDir},
      {M::WalkToPos, &WalkToPos},
      {M::Sufi, &Sufi},
  };
};
