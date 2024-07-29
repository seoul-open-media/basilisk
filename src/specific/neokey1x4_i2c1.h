#pragma once

#include <Adafruit_NeoKey_1x4.h>
#include <Wire.h>
#include <neokey.h>

// Assuming specific setup:
// I2C1        col 0
// row y         x    0 1 2 3  addr
//   0 0  _neokeys[0]{0 1 2 3} 0x30

#ifdef NEOKEY_DIM_X
#undef NEOKEY_DIM_X
#endif
#define NEOKEY_DIM_X 4

#ifdef NEOKEY_DIM_Y
#undef NEOKEY_DIM_Y
#endif
#define NEOKEY_DIM_Y 1

#ifndef I2C_BUS1
#define I2C_BUS1 (&Wire1)
#endif

namespace specific {

Adafruit_NeoKey_1x4 neokey1x4_i2c1_mtx[NEOKEY_DIM_Y][NEOKEY_DIM_X / 4] = {
    Adafruit_NeoKey_1x4{0x30, I2C_BUS1}};

Neokey neokey1x4_i2c1{(Adafruit_NeoKey_1x4*)neokey1x4_i2c1_mtx,  //
                      NEOKEY_DIM_Y, NEOKEY_DIM_X / 4};

}  // namespace specific
