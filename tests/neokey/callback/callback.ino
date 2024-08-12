#include <beat.h>
#include <initializers.h>
#include <specific/neokey3x4_i2c1.h>

auto& neokey = specific::neokey3x4_i2c1;

Beat neokey_beat{10};

void print(uint16_t key) {
  Serial.print(F("Key rose: "));
  Serial.println(key);
}

void setup() {
  SerialInitializer.init();
  I2C1Initializer.init();
  NeokeyInitializer.init(neokey);
  neokey.SetCommonRiseCallback(print);
}

void loop() {
  if (neokey_beat.Hit()) neokey.Read();
}
