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
  static void Walk(Basilisk*);
  // static void Diamond(Basilisk*);
  // static void Gee(Basilisk*);

  inline static const std::map<M, void (*)(Basilisk*)> mode_runners = {
      {M::Idle_Init, &Idle},
      {M::Idle_Nop, &Idle},
      {M::Wait, &Wait},
      {M::Free, &Free},
      {M::SetMags, &SetMags},
      {M::SetPhis_Init, &SetPhis},
      {M::SetPhis_Stop, &SetPhis},
      {M::Pivot_Init, &Pivot},
      {M::Pivot_Kick, &Pivot},
      {M::Walk, &Walk}  //,
                        // {M::Diamond_Init, &Diamond},
                        // {M::Diamond_Step, &Diamond},
                        // {M::Gee_Init, &Gee},
                        // {M::Gee_Step, &Gee}
  };
};
