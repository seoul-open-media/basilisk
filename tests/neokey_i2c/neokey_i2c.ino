// Assuming Adafruit NeoKey1x4 is connected to I2C0,
// continuously print to Serial the NeoKey1x4 reading.

#include <Adafruit_NeoKey_1x4.h>
#include <Wire.h>

#define NEOKEY_I2C_ADDRESS 0x30
Adafruit_NeoKey_1x4 neokey{NEOKEY_I2C_ADDRESS,
                           &Wire};  // Wire = I2C0; Wire1 = I2C1

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    Serial.println(F("Serial begin failed"));
    delay(1000);
  }
  Serial.println(F("Serial started"));

  Wire.begin();
  Serial.println(F("I2C0 started"));

  while (!neokey.begin()) {
    Serial.println(F("NeoKey begin failed"));
    delay(1000);
  }
  Serial.println(F("NeoKey started"));
  delay(1000);
}

void loop() {
  auto reading = neokey.read();

  Serial.print(F("Reading in binary: "));
  Serial.println(reading, BIN);
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(F("Button "));
    Serial.print(i);
    Serial.print(F(": "));
    Serial.println(reading & 0x1 ? "Pressed" : "Not Pressed");
    reading >>= 1;
  }

  delay(500);
}
