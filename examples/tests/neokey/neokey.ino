// Print to Serial in 4Hz frequency the reading from a matrix of
// Adafruit NeoKey1x4s connected to I2C bus 1.
// The specific configuration of the matrix is described in
// `src/specific/neokey4x4_i2c1.h`. You can use other configurations by
// #including one of them.
//
// Learn how to set I2C addresses of NeoKeys from the following source:
// https://learn.adafruit.com/neokey-1x4-qt-i2c/pinouts#address-jumpers-3098419

#include <Metro.h>
#include <TeensyThreads.h>
#include <initializers.h>
#include <neokey.h>
#include <specific/neokey3x4_i2c1.h>

auto& neokey = specific::neokey3x4_i2c1;

class NeokeyReceiver {
 public:
  NeokeyReceiver(Neokey& neokey) : neokey_{neokey} {}

  void Run(const uint32_t& interval = 10) {  // Default 100Hz.
    Metro metro{interval};
    while (1) {
      if (metro.check()) {
        neokey_.read();
      }
    }
  }

  Neokey& neokey_;
} neokey_r{neokey};

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

void setup() {
  SerialInitializer.init();
  I2C1Initializer.init();
  NeokeyInitializer.init(neokey);
  neokey.registerCallbackAll(print);

  threads.addThread([] { neokey_r.Run(); });
}

void loop() {}
