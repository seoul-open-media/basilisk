#include <Wire.h>

bool toggles[3];

void setup() {
  Serial.begin(9600);
  Wire.begin();
}

void loop() {
  delay(10);

  if (!Serial.available()) return;

  String serial_msg = "";
  while (Serial.available()) {
    serial_msg += (char)Serial.read();
  }
  Serial.print(F("serial_msg = "));
  Serial.println(serial_msg);

  auto len = serial_msg.length() - 1;
  if (len < 1 || len > 3) return;

  byte i2c_dest_addr = len;
  Serial.print(F("i2c_dest_addr = "));
  Serial.println(i2c_dest_addr);

  auto& toggle = toggles[i2c_dest_addr - 1];
  toggle = !toggle;
  byte i2c_msg = toggle ? 1 : 0;
  Serial.print(F("i2c_msg = "));
  Serial.println(i2c_msg);

  Wire.beginTransmission(i2c_dest_addr);
  Wire.write(i2c_msg);
  Wire.endTransmission();

  Serial.println();
}
