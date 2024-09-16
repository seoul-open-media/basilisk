#pragma once

#include "mode_runners.h"

void ModeRunners::SetMags(Basilisk* b) {
  auto& m = b->cmd_.mode;
  auto& c = b->cmd_.set_mags;

  switch (m) {
    case M::SetMags: {
      Serial.println("ModeRunners::SetMags");

      for (uint8_t id = 0; id < 4; id++) {
        b->mags_.SetStrength(id, c.strengths[id]);
      }

      m = M::Wait;
      b->cmd_.wait.exit_condition = [](Basilisk* b) {
        if (millis() - b->cmd_.wait.init_time <
            b->cmd_.set_mags.min_wait_time) {
          return false;
        }
        if (millis() - b->cmd_.wait.init_time >
            b->cmd_.set_mags.max_wait_time) {
          return true;
        }

        for (uint8_t lr = 0; lr < 2; lr++) {
          if (b->cmd_.set_mags.expected_contact[lr]) {
            if (!b->lego_.state_[lr].ConsecutiveContact(
                    b->cmd_.set_mags.verif_thr)) {
              return false;
            }
          } else {
            if (!b->lego_.state_[lr].ConsecutiveDetachment(
                    b->cmd_.set_mags.verif_thr)) {
              return false;
            }
          }
        }

        return true;
      };
      b->cmd_.wait.exit_to_mode = c.exit_to_mode;
      b->cmd_.wait.init_time = millis();
    } break;
    default:
      break;
  }
}
