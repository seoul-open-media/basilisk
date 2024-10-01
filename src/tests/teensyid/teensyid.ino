#include <Arduino.h>

void setup() {
  Serial.begin(9600);
  while (!Serial);

  volatile uint32_t* uuid0 = (volatile uint32_t*)0x401F4410;

  Serial.print("Unique ID: ");
  for (int i = 0; i < 8; i++) {
    Serial.print(uuid0[i], HEX);
    if (i < 7) Serial.print(" - ");
  }
  Serial.println();
}

void loop() {}
