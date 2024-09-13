#include "helpers/imports.h"
#include "helpers/utils.h"
#include "rpl_sndrs/serial_rs.h"
#include "servo_units/basilisk.h"

Basilisk b{Basilisk::Configuration{}};

utils::Beat serial_plotter_rs_beat{100};

void setup() {
  Serial.begin(9600);
  while (!Serial);
  b.Setup();

  Serial.println(b.cfg_.servo.bus);
}

void loop() {
  b.Run();

  if (serial_plotter_rs_beat.Hit()) {
    SerialReplySender(b);
  }
}
