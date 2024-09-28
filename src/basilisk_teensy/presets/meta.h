#pragma once

#include "../servo_units/basilisk.h"

struct Presets {
  using M = Basilisk::Command::Mode;

  inline static void Idle(Basilisk* b) { b->cmd_.mode = M::Idle_Init; }
  static void RMagRls(Basilisk*);
  static void RMagAtt(Basilisk*);
  static void LMagRls(Basilisk*);
  static void LMagAtt(Basilisk*);
  static void RandomMagsWeak(Basilisk*);
  static void RandomMagsStrong(Basilisk*);
  static void PivRFr45(Basilisk*);
  static void PivLFr45(Basilisk*);
  static void PivRBk45(Basilisk*);
  static void PivLBk45(Basilisk*);
  static void PivRFr90(Basilisk*);
  static void PivLFr90(Basilisk*);
  static void PivRBk90(Basilisk*);
  static void PivLBk90(Basilisk*);
  static void BendRIn45(Basilisk*);
  static void BendROut45(Basilisk*);
  static void BendLIn45(Basilisk*);
  static void BendLOut45(Basilisk*);

  inline static const std::map<uint16_t, void (*)(Basilisk*)> presets = {
      {50000, &Idle},          {1, &RMagRls},     {2, &RMagAtt},
      {3, &LMagRls},           {4, &LMagAtt},     {5, &PivRFr45},
      {6, &PivLFr45},          {7, &PivRBk45},    {8, &PivLBk45},
      {9, &PivRFr90},          {10, &PivLFr90},   {11, &PivRBk90},
      {12, &PivLBk90},         {13, &BendRIn45},  {14, &BendROut45},
      {15, &BendLIn45},        {16, &BendLOut45}, {23, &RandomMagsWeak},
      {24, &RandomMagsStrong},
  };
};
