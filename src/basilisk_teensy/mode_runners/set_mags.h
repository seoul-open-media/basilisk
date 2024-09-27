#pragma once

#include "meta.h"

void ModeRunners::SetMags(Basilisk* b) {
  static uint32_t init_time;

  auto& m = b->cmd_.mode;
  auto& c = b->cmd_.set_mags;

  switch (m) {
    case M::SetMags_Init: {
      // Serial.println("ModeRunners::SetMags(Init)");

      for (uint8_t id = 0; id < 4; id++) {
        b->mags_.SetStrength(id, c.strengths[id]);
      }
      init_time = millis();
      m = M::SetMags_Wait;
    } break;
    case M::SetMags_Wait: {
      // Serial.println("ModeRunners::SetMags(Wait)");

      if ([&] {
            if (millis() - init_time > c.max_dur) {
              return true;
            }
            if (millis() - init_time < c.min_dur) {
              return false;
            }
            for (const uint8_t f : IDX_LR) {
              if (c.expected_state[f]) {
                if (!b->lego_.state_[f].ConsecutiveContact(c.verif_thr)) {
                  return false;
                }
              } else {
                if (!b->lego_.state_[f].ConsecutiveDetachment(c.verif_thr)) {
                  return false;
                }
              }
            }
            return true;
          }()) {
        m = c.exit_to_mode;
      };
    } break;
    default:
      break;
  }
}
