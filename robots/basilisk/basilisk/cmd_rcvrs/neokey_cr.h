#pragma once

#include <specific/neokey3x4_i2c0.h>

auto& neokey = specific::neokey3x4_i2c0;

uint16_t neokey_cmd = 0;

void neokey_cb(uint16_t key) {
  Serial.print(F("Neokey rose: "));
  Serial.println(key + 1);
  neokey_cmd = key + 1;
}

class NeokeyCommandReceiver {
 public:
  void Setup() { neokey.SetCommonRiseCallback(neokey_cb); }

  void Run() { neokey.Read(); }
} neokey_cr;
