// Print to Serial in 4Hz frequency the reading from a matrix of
// Adafruit NeoKey1x4s connected to I2C0. Assuming just a 1x1 matrix for now.
// Learn how to set I2C addresses of NeoKeys from the following source:
// https://learn.adafruit.com/neokey-1x4-qt-i2c/pinouts#address-jumpers-3098419

#include <Adafruit_NeoKey_1x4.h>
#include <Metro.h>
#include <TeensyThreads.h>
#include <Wire.h>
#include <initializers.h>
#include <neokey.h>

// Assume the following setup:
// I2C0        col 0
// row y         x    0 1 2 3  addr
//   0 0  _neokeys[0]{K K K K} 0x30

#define NEOKEY_DIM_X 4
#define NEOKEY_DIM_Y 1

Adafruit_NeoKey_1x4 neokey_mtx[NEOKEY_DIM_Y][NEOKEY_DIM_X / 4] = {
    Adafruit_NeoKey_1x4{0x30, &Wire}};

// The way NeoKey's callback is programmed is weird,
// but just use it for now since it works anyway.
NeoKey1x4Callback print(keyEvent evt) {
  auto key = evt.bit.NUM;
  if (evt.bit.EDGE == SEESAW_KEYPAD_EDGE_RISING) {
    Serial.print("Rise: ");
    Serial.println(key);
  } else if (evt.bit.EDGE == SEESAW_KEYPAD_EDGE_FALLING) {
    Serial.print("Fall: ");
    Serial.println(key);
  }

  return 0;
}

class NeokeyReceiver {
 public:
  NeokeyReceiver(Adafruit_NeoKey_1x4* neokeys, uint8_t rows, uint8_t cols)
      : neokey_{neokeys, rows, cols} {}

  void Run(const uint32_t& interval = 10) {
    Metro metro{interval};
    while (1) {
      if (metro.check()) {
        neokey_.read();
      }
    }
  }

  Neokey neokey_;
} neokey_r{(Adafruit_NeoKey_1x4*)neokey_mtx, NEOKEY_DIM_Y, NEOKEY_DIM_X / 4};

void setup() {
  SerialInitializer.init();
  I2C0Initializer.init();

  neokey_r.neokey_.begin();
  for (uint8_t y = 0; y < NEOKEY_DIM_Y; y++) {
    for (uint8_t x = 0; x < NEOKEY_DIM_X; x++) {
      neokey_r.neokey_.registerCallback(x, y, print);
    }
  }

  threads.addThread([] { neokey_r.Run(); });
}

void loop() {}
