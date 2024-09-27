#pragma once

#include "../presets/matome.h"
#include "meta.h"

void ModeRunners::DoPreset(Basilisk* b) {
  auto& m = b->cmd_.mode;
  auto& c = b->cmd_.do_preset;

  switch (m) {
    case M::DoPreset: {
      // Serial.println("ModeRunners::DoPreset");

      auto* maybe_preset = SafeAt(Presets::presets, c.idx);
      if (maybe_preset) {
        (*maybe_preset)(b);
      } else {
        Serial.println("Unregistered preset index");
        m = M::Idle_Init;
      }
    } break;
    default:
      break;
  }
}
