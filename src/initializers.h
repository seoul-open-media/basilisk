#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <neokey.h>
#include <servo.h>

template <typename InitParam>
class Initializer {
 public:
  Initializer(void (*init)(InitParam)) : init_{init}, done_{false} {}

  void init(InitParam arg) {
    if (!done_ && init_) {
      init_(arg);
      done_ = true;
    }
  }

 private:
  void (*init_)(InitParam);
  bool done_;
};

template <>
class Initializer<void> {
 public:
  Initializer(void (*init)()) : init_{init}, done_{false} {}

  void init() {
    if (!done_ && init_) {
      init_();
      done_ = true;
    }
  }

 private:
  void (*init_)();
  bool done_;
};

Initializer<void> SerialInitializer{[] {
  Serial.begin(115200);
  while (!Serial) {
    Serial.println(F("Serial begin failed"));
    delay(1000);
  }
  Serial.println(F("Serial started"));
}};

Initializer<void> SpiInitializer{[] {
  SPI.begin();
  Serial.println(F("SPI started"));
}};

Initializer<void> Spi1Initializer{[] {
  SPI1.begin();
  Serial.println(F("SPI1 started"));
}};

Initializer<void> I2C0Initializer{[] {
  Wire.begin();
  Serial.println(F("I2C0(Wire) started"));
}};

Initializer<void> I2C1Initializer{[] {
  Wire1.begin();
  Serial.println(F("I2C1(Wire1) started"));
}};

Initializer<Neokey&> NeokeyInitializer{[](Neokey& neokey) {
  while (!neokey.begin()) {
    Serial.println(F("Neokey begin failed"));
    delay(1000);
  }
}};

Initializer<uint8_t> CanFdInitializer{[](uint8_t bus) {
  void (*isr)();
  // Capturing `bus` for a lambda disables cast to `void(*)()`.
  switch (bus) {
    case 1: {
      isr = [] { canfd_drivers[0].isr(); };
    } break;
    case 2: {
      isr = [] { canfd_drivers[1].isr(); };
    } break;
    case 3: {
      isr = [] { canfd_drivers[2].isr(); };
    } break;
    case 4: {
      isr = [] { canfd_drivers[3].isr(); };
    } break;
    default: {
      while (1) {
        Serial.print(F("Unknown bus: "));
        Serial.println(bus);
        delay(1000);
      }
    }
  }

  while (1) {
    auto err_code = canfd_drivers[bus - 1].begin(
        [] {
          ACAN2517FDSettings settings{ACAN2517FDSettings::OSC_40MHz,
                                      1000ll * 1000ll, DataBitRateFactor::x1};
          settings.mArbitrationSJW = 2;
          settings.mDriverTransmitFIFOSize = 1;
          settings.mDriverReceiveFIFOSize = 2;

          return settings;
        }(),
        isr);

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
