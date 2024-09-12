#pragma once

#include "../components/neokey.h"
#include "../servo_units/basilisk.h"

namespace basilisk {

class NeokeyCommandReceiver {
 public:
  NeokeyCommandReceiver(Basilisk& b, Neokey& nk) : basilisk_{b}, neokey_{nk} {}

  void Setup() {
    neokey.SetCommonRiseCallback([](uint16_t key) {
      Serial.print(F("Neokey rose: "));
      Serial.println(key + 1);
      neokey_cmd = key + 1;
    });
  }

  void Run() { neokey.Run(); }

 private:
  Neokey& neokey_;
  Basilisk& basilisk_;
};

}  // namespace basilisk
