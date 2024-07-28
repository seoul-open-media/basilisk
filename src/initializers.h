#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <neokey.h>
#include <servo.h>

class Initializer {
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

Initializer SerialInitializer{[] {
  Serial.begin(115200);
  while (!Serial) {
    Serial.println(F("Serial begin failed"));
    delay(1000);
  }
  Serial.println(F("Serial started"));
}};

Initializer SpiInitializer{[] {
  SPI.begin();
  Serial.println(F("SPI started"));
}};

Initializer I2C0Initializer{[] {
  Wire.begin();
  Serial.println(F("I2C0(Wire) started"));
}};

Initializer I2C1Initializer{[] {
  Wire1.begin();
  Serial.println(F("I2C1(Wire1) started"));
}};

Initializer CanFdInitializer{[] {  // Caution: Only for Bus 1 (JC1 port)
  while (1) {
    auto err_code = canfd_driver.begin(
        [] {
          ACAN2517FDSettings settings{ACAN2517FDSettings::OSC_40MHz,
                                      1000ll * 1000ll, DataBitRateFactor::x1};
          settings.mArbitrationSJW = 2;
          settings.mDriverTransmitFIFOSize = 1;
          settings.mDriverReceiveFIFOSize = 2;
          return settings;
        }(),
        [] { canfd_driver.isr(); });
    if (!err_code) {
      Serial.println("CAN FD driver started");
      break;
    }
    Serial.print(F("CAN FD driver begin failed, error code 0x"));
    Serial.println(err_code, HEX);
    delay(1000);
  }
}};
