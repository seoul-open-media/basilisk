#include <Metro.h>
#include <TeensyThreads.h>
#include <initializers.h>
#include <neokey.h>
#include <specific/neokey3x4_i2c1.h>

auto& neokey = specific::neokey3x4_i2c1;

void Receive(const uint16_t& interval) {
  Metro metro{interval};
  while (1) {
    if (metro.check()) {
      neokey.read();
    }
  }
}

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
  threads.addThread([] { Receive(10); });
}

void loop() { yield(); }
