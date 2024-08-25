#include <Wire.h>

int led_pin = 13;

void setup() {
  pinMode(led_pin, OUTPUT);
  Wire.begin(2);
  Wire.onReceive(toggle);
}

void loop() {
  delay(100);
}

void toggle(int dummy) {
  Serial.print(F("dummy = "));
  Serial.println(dummy);

  byte i2c_msg = Wire.read();
  if (i2c_msg == 1) {
    digitalWrite(led_pin, HIGH);
  } else if (i2c_msg == 0) {
    digitalWrite(led_pin, LOW);
  }
}
