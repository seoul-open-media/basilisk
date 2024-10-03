#pragma once

#include "../cmd_rcvrs/xbee_cr.h"
#include "../servo_units/basilisk.h"

struct Presets {
  using M = Basilisk::Command::Mode;

  inline static void Idle(Basilisk* b) { b->cmd_.mode = M::Idle_Init; }
  inline static void Free(Basilisk* b) { b->cmd_.mode = M::Free; }

  inline static void CRMuxXbee(Basilisk* b) {
    // Executer handles me.
  }
  inline static void SetBaseYawZero(Basilisk* b) {
    b->cmd_.oneshots |= (1 << 1);
    b->cmd_.set_base_yaw.offset = 0.0;
    b->cmd_.mode = M::Idle_Init;
  }
  inline static void SetBaseYawM025(Basilisk* b) {
    b->cmd_.oneshots |= (1 << 1);
    b->cmd_.set_base_yaw.offset = -0.25;
    b->cmd_.mode = M::Idle_Init;
  }

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
      {50000, &Idle},
      {50001, &Free},

      {50002, &CRMuxXbee},
      {50003, &SetBaseYawZero},
      {50004, &SetBaseYawM025},

      {1, &RMagRls},
      {2, &RMagAtt},
      {3, &LMagRls},
      {4, &LMagAtt},
      {23, &RandomMagsWeak},
      {24, &RandomMagsStrong},

      {5, &PivRFr45},
      {6, &PivLFr45},
      {7, &PivRBk45},
      {8, &PivLBk45},
      {9, &PivRFr90},
      {10, &PivLFr90},
      {11, &PivRBk90},
      {12, &PivLBk90},
      {13, &BendRIn45},
      {14, &BendROut45},
      {15, &BendLIn45},
      {16, &BendLOut45},
  };
};
