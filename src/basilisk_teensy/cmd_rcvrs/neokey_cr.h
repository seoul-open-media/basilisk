#pragma once

#include "../components/neokey.h"
#include "../servo_units/basilisk.h"

class NeokeyCommandReceiver {
 public:
  NeokeyCommandReceiver(Basilisk& b, Neokey& nk) : b_{b}, nk_{nk} {}

  // Should be called before use.
  bool Setup() {
    if (!nk_.Setup([](uint16_t key) {
          Serial.print(F("Neokey rose: "));
          Serial.println(key + 1);
          nk_cmd = key + 1;
        })) {
      Serial.println("Neokey: Setup failed");
      return false;
    };

    Serial.println("Neokey: Setup complete");
    return true;
  }

  void Run() { nk_.Run(); }

 private:
  inline static uint16_t nk_cmd = 0;
  Neokey& nk_;
  Basilisk& b_;
};
