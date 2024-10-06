#pragma once

#include <Arduino.h>

#include "../components/neokey.h"

void LedReplySender(Neokey& nk) {
  static bool on = true;

  for (uint8_t i = 0; i < 4; i++) nk.setPixelColor(i, on ? 0x000020 : 0x000000);
  nk.show();
  on = !on;
}
