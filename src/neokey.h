#pragma once

#include <Adafruit_NeoKey_1x4.h>
#include <Arduino.h>

// Definition of row, col, x, y in a matrix of NeoKey1x4s:
//             col 0            1            2
// row y         x    0 1 2 3      4 5 6 7      8 9 a b
//   0 0  _neokeys[0]{K K K K} [1]{K K K K} [2]{K K K K}
//   1 1          [3]{K K K K} [4]{K K K K} [5]{K K K K}
//   2 2          [6]{K K K K} [7]{K K K K} [8]{K K K K}

// Just a wrapper class of Adafruit_MultiNeoKey1x4 with some
// helper methods added and callback handling done right.
class Neokey : private Adafruit_MultiNeoKey1x4 {
 public:
  Neokey(Adafruit_NeoKey_1x4* neokeys, uint8_t rows, uint8_t cols)
      : Adafruit_MultiNeoKey1x4{neokeys, rows, cols} {
    last_buttons_ = new uint8_t[_rows * _cols];
    memset(last_buttons_, 0, _cols * _rows);
  }

  ~Neokey() { delete[] last_buttons_; }

  bool begin() { return static_cast<Adafruit_MultiNeoKey1x4*>(this)->begin(); }

  uint8_t dim_y() { return _rows; }
  uint8_t dim_x() { return _cols << 2; }

  void SetCommonRiseCallback(void (*callback)(uint16_t)) {
    common_rise_callback_ = callback;
  }

  void Read() {
    for (uint8_t row = 0; row < _rows; row++) {
      for (uint8_t col = 0; col < _cols; col++) {
        auto nk_idx = row * _cols + col;
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

        for (uint8_t b = 0; b < 4; b++) {
          if (just_pressed & (1 << b) && common_rise_callback_) {
            uint16_t key = (nk_idx << 2) + b;
            common_rise_callback_(key);
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
