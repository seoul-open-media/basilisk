#pragma once

#include <Adafruit_NeoKey_1x4.h>
#include <Arduino.h>

// Definition of row, col, x, y in a matrix of NeoKey1x4s:
//             col 0            1            2
// row y         x    0 1 2 3      4 5 6 7      8 9 a b
//   0 0  _neokeys[0]{K K K K} [1]{K K K K} [2]{K K K K}
//   1 1          [3]{K K K K} [4]{K K K K} [5]{K K K K}
//   2 2          [6]{K K K K} [7]{K K K K} [8]{K K K K}

// Just a wrapper class with individual read and print method added.
class Neokey : public Adafruit_MultiNeoKey1x4 {
 public:
  using Adafruit_MultiNeoKey1x4::read;

  Neokey(Adafruit_NeoKey_1x4* neokeys, uint8_t rows, uint8_t cols)
      : Adafruit_MultiNeoKey1x4{neokeys, rows, cols} {}

  uint8_t read(uint8_t row, uint8_t col, bool do_print) {
    auto reading = _neokeys[row * _cols + col].read();
    if (do_print) {
      Serial.print(F("NeoKey[row = "));
      Serial.print(row);
      Serial.print(F("][col = "));
      Serial.print(col);
      Serial.print(F("] = 0b"));
      Serial.println(reading, BIN);
      for (uint8_t i = 0; i < 4; i++) {
        Serial.print(F("Key["));
        Serial.print(i);
        Serial.print(F("] "));
        Serial.println(reading & 0x1 ? "Pressed" : "Not Pressed");
        reading >>= 1;
      }
    }
    return reading;
  }
};
