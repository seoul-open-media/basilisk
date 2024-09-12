#include "components/footswitches.h"
#include "components/imu.h"
#include "components/lps.h"
#include "helpers/imports.h"
#include "helpers/utils.h"

FootSwitches fsw{23, 29};
Imu imu{};
Lps lps{300.0, 300.0, 300.0};
utils::Beat fsw_beat{10};
utils::Beat serial_plotter_rs_beat{100};

void setup() {
  Serial.begin(9600);
  imu.Setup();
  lps.Setup();
  fsw.Setup();
}

void loop() {
  {
    imu.Run();
    lps.Run();
  }

  if (fsw_beat.Hit()) {
    fsw.Run();
  }

  if (serial_plotter_rs_beat.Hit()) {
    Serial.print("fsw.state_[0].contact:");
    Serial.print(fsw.state_[0].contact, BIN);
    Serial.print(",");
    Serial.print("fsw.state_[1].contact:");
    Serial.print(fsw.state_[1].contact, BIN);
    Serial.println();

    Serial.print("fsw.state_[0].Contact(32):");
    Serial.print(fsw.state_[0].Contact(32) ? "True" : "False");
    Serial.print(",");
    Serial.print("fsw.state_[1].Contact(32):");
    Serial.print(fsw.state_[1].Contact(32) ? "True" : "False");
    Serial.println();

    Serial.print("imu.euler_[0]:");
    Serial.print(imu.euler_[0]);
    Serial.print(",");
    Serial.print("imu.euler_[1]:");
    Serial.print(imu.euler_[1]);
    Serial.print(",");
    Serial.print("imu.euler_[2]:");
    Serial.print(imu.euler_[2]);
    Serial.println();

    Serial.print("lps.x_:");
    Serial.print(lps.x_);
    Serial.print(",");
    Serial.print("lps.y_:");
    Serial.print(lps.y_);
    Serial.println();
  }
}
