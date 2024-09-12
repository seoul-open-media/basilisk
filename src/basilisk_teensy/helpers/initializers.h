#pragma once

#include "../components/canfd_drivers.h"
#include "../components/neokey.h"
#include "imports.h"

namespace initializers {

template <typename InitFunc>
class Initializer {
 public:
  Initializer(InitFunc init) : init_{init}, done_{false} {}

  void Init() {
    if (!done_) {
      init_();
      done_ = true;
    }
  }

  template <typename InitParam>
  void Init(InitParam arg) {
    if (!done_) {
      init_(arg);
      done_ = true;
    }
  }

 private:
  InitFunc init_;
  bool done_ = false;
};

Initializer serial{[] {
  Serial.begin(115200);
  if (!!Serial) Serial.println(F("Serial started"));
}};

Initializer spi0{[] {
  SPI.begin();
  Serial.println(F("SPI started"));
}};

Initializer spi1{[] {
  SPI1.begin();
  Serial.println(F("SPI1 started"));
}};

Initializer i2c0{[] {
  Wire.begin();
  Serial.println(F("I2C0(Wire) started"));
}};

Initializer i2c1{[] {
  Wire1.begin();
  Serial.println(F("I2C1(Wire1) started"));
}};

Initializer neokey{[](Neokey& nk) {
  while (!nk.Setup()) {
    Serial.println(F("Neokey begin failed"));
    delay(1000);
  }
  Serial.println(F("Neokey started"));
}};

Initializer canfd_driver{[](const uint8_t bus) {
  if (bus < 1 || bus > 4) {
    while (1) {
      Serial.print(F("Unknown bus: "));
      Serial.println(bus);
      delay(1000);
    }
  }

  void (*isr[4])() = {
      [] { canfd_drivers[0].isr(); }, [] { canfd_drivers[1].isr(); },
      [] { canfd_drivers[2].isr(); }, [] { canfd_drivers[3].isr(); }};

  while (1) {
    const auto err_code = canfd_drivers[bus - 1].begin(
        [] {
          ACAN2517FDSettings settings{ACAN2517FDSettings::OSC_40MHz,
                                      1000ll * 1000ll, DataBitRateFactor::x1};
          settings.mArbitrationSJW = 2;
          settings.mDriverTransmitFIFOSize = 1;
          settings.mDriverReceiveFIFOSize = 2;

          return settings;
        }(),
        isr[bus - 1]);

    if (!err_code) {
      Serial.print(F("CAN FD driver on bus "));
      Serial.print(bus);
      Serial.println(F(" started"));
      return;
    }

    Serial.print(F("CAN FD driver on bus "));
    Serial.print(bus);
    Serial.print(F(" begin failed, error code 0x"));
    Serial.println(err_code, HEX);

    delay(1000);
  }
}};

}  // namespace initializers
