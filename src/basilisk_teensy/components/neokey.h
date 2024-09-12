#pragma once

#include "../helpers/imports.h"

// Definition of row, col, x, y in a matrix of NeoKey1x4s:
//             col 0            1            2
// row y         x    0 1 2 3      4 5 6 7      8 9 a b
//   0 0  _neokeys[0]{K K K K} [1]{K K K K} [2]{K K K K}
//   1 1          [3]{K K K K} [4]{K K K K} [5]{K K K K}
//   2 2          [6]{K K K K} [7]{K K K K} [8]{K K K K}

// A wrapper class of Adafruit_MultiNeoKey1x4, callback handling done right.
class Neokey : private Adafruit_MultiNeoKey1x4 {
 public:
  Neokey(Adafruit_NeoKey_1x4* neokeys, uint8_t rows, uint8_t cols)
      : Adafruit_MultiNeoKey1x4{neokeys, rows, cols} {
    last_buttons_ = new uint8_t[_rows * _cols];
    memset(last_buttons_, 0, _rows * _cols);
  }

  ~Neokey() { delete[] last_buttons_; }

  bool Setup(void (*callback)(uint16_t)) {
    Wire.begin();

    if (!begin()) {
      Serial.println("Neokey: Begin failed");
      return false;
    }
    Serial.println("Neokey: Started");

    if (!callback) {
      Serial.println("Neokey: Common rise callback register failed");
      return false;
    }
    common_rise_callback_ = callback;
    Serial.println("Neokey: Common rise callback registered");

    Serial.println("Neokey: Setup complete");
    return true;
  }

  // Should be called in regular interval short enough to ensure that
  // no physical press of a button is missed.
  void Run() {
    if (!setup_cplt_) {
      Serial.println("Neokey: Setup NOT complete");
      return;
    }

    for (uint8_t row = 0; row < _rows; row++) {
      for (uint8_t col = 0; col < _cols; col++) {
        const uint8_t nk_idx = row * _cols + col;
        auto& nk = _neokeys[nk_idx];

        // "Not sure why we have to do it twice."
        nk.digitalReadBulk(NEOKEY_1X4_BUTTONMASK);
        auto buttons = nk.digitalReadBulk(NEOKEY_1X4_BUTTONMASK);
        buttons ^= NEOKEY_1X4_BUTTONMASK;
        buttons &= NEOKEY_1X4_BUTTONMASK;
        buttons >>= NEOKEY_1X4_BUTTONA;

        // Compare to last reading.
        auto& last_buttons = last_buttons_[nk_idx];
        uint8_t just_pressed = (buttons ^ last_buttons) & buttons;

        // Call callback for risen buttons.
        for (uint8_t b = 0; b < 4; b++) {
          if (just_pressed & (1 << b)) {
            common_rise_callback_((nk_idx << 2) + b);
          }
        }

        // Stash for next run.
        last_buttons = buttons;
      }
    }
  }

 private:
  void (*common_rise_callback_)(uint16_t key);
  uint8_t* last_buttons_;
};
