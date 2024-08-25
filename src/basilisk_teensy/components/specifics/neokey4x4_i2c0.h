#pragma once

#include "../neokey.h"

// Assuming specific setup:
// I2C0        col 0
// row y         x    0 1 2 3  addr
//   0 0  _neokeys[0]{0 1 2 3} 0x30
//   1 1  _neokeys[1]{4 5 6 7} 0x31
//   2 2  _neokeys[2]{8 9 a b} 0x32
//   3 3  _neokeys[3]{c d e f} 0x33

#ifdef NEOKEY_DIM_X
#undef NEOKEY_DIM_X
#endif
#define NEOKEY_DIM_X 4

#ifdef NEOKEY_DIM_Y
#undef NEOKEY_DIM_Y
#endif
#define NEOKEY_DIM_Y 4

#ifndef I2C_BUS0
#define I2C_BUS0 (&Wire)
#endif

namespace basilisk {
namespace specifics {

Adafruit_NeoKey_1x4 neokey4x4_i2c0_mtx[NEOKEY_DIM_Y][NEOKEY_DIM_X / 4] = {
    Adafruit_NeoKey_1x4{0x30, I2C_BUS},  //
    Adafruit_NeoKey_1x4{0x31, I2C_BUS},  //
    Adafruit_NeoKey_1x4{0x32, I2C_BUS},  //
    Adafruit_NeoKey_1x4{0x33, I2C_BUS}};

Neokey neokey4x4_i2c0{(Adafruit_NeoKey_1x4*)neokey4x4_i2c0_mtx,  //
                      NEOKEY_DIM_Y, NEOKEY_DIM_X / 4};

}  // namespace specifics
}  // namespace basilisk
