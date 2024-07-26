// Print to Serial in 4Hz frequency the reading from
// Adafruit NeoKey1x4 connected to I2C0.

#include <Adafruit_NeoKey_1x4.h>
#include <Metro.h>
#include <Wire.h>
#include <initializers.h>
#include <neokey.h>

void setup() {
  SerialInitializer.init();
  I2C0Initializer.init();
  Neokey0Initializer.init();
}

Metro metro{250};

void loop() {
  if (metro.check()) {
    NeokeyPrint(neokey0.read());
  }
}
