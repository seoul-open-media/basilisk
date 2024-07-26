#pragma once

#include <Adafruit_NeoKey_1x4.h>
#include <Arduino.h>

#define NEOKEY_I2C_ADDRESS 0x30

Adafruit_NeoKey_1x4 neokey0{NEOKEY_I2C_ADDRESS, &Wire};
Adafruit_NeoKey_1x4 neokey1{NEOKEY_I2C_ADDRESS, &Wire1};

void NeokeyPrint(uint8_t reading) {
  Serial.print(F("Reading value in binary: "));
  Serial.println(reading, BIN);
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(F("Button "));
    Serial.print(i);
    Serial.print(F(": "));
    Serial.println(reading & 0x1 ? "Pressed" : "Not Pressed");
    reading >>= 1;
  }
}
