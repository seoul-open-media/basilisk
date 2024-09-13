#pragma once

#include <map>

#include "../servo_units/basilisk.h"

struct ModeRunners {
  using M = Basilisk::Command::Mode;

  static void Idle(Basilisk*);
  static void Wait(Basilisk*);
  static void Free(Basilisk*);
  static void SetRho(Basilisk*);
  static void Walk(Basilisk*);
  static void Diamond(Basilisk*);
  static void Gee(Basilisk*);

  inline static const std::map<M, void (*)(Basilisk*)> mode_runners = {
      {M::Idle_Init, &Idle},
      {M::Idle_Nop, &Idle},
      {M::Wait, &Wait},
      {M::Free, &Free},
      {M::SetRho, &SetRho},
      {M::Walk_Init, &Walk},
      {M::Walk_InitLeft, &Walk},
      {M::Walk_InitRight, &Walk},
      {M::Walk_Step, &Walk},
      {M::Diamond_Init, &Diamond},
      {M::Diamond_Step, &Diamond},
      {M::Gee_Init, &Gee},
      {M::Gee_Step, &Gee}};
};
