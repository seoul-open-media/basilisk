#pragma once

#include "meta.h"

void ModeRunners::RandomMags(Basilisk* b) {
  static uint32_t dur[4] = {0, 0, 0, 0};

  auto& m = b->cmd_.mode;
  auto& c = b->cmd_.random_mags;

  switch (m) {
    case M::RandomMags: {
      // Serial.println("ModeRunners::RandomMags");

      for (uint8_t id = 0; id < 4; id++) {
        if (dur[id] == 0) {
          randomSeed(b->cfg_.suid * 100 + id * millis());
          dur[id] = random(c.min_phase_dur, c.max_phase_dur);
        } else {
          const uint32_t last_switch_time =
              b->mags_.attaching_[id] ? b->mags_.last_release_time_[id]
                                      : b->mags_.last_attach_time_[id];
          if (millis() - last_switch_time >= dur[id]) {
            b->mags_.SetStrength(id, Bool2MS(!b->mags_.attaching_[id]));
            dur[id] = 0;
          }
        }
      }
    } break;
    default:
      break;
  }
}
