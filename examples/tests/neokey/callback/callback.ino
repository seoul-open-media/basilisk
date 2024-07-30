#include <Metro.h>
#include <initializers.h>
#include <neokey.h>
#include <specific/neokey3x4_i2c1.h>

auto& neokey = specific::neokey3x4_i2c1;

Metro receive_metro{10};
void Receive() { neokey.read(); }

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
}

void loop() {
  if (receive_metro.check()) Receive();
}
