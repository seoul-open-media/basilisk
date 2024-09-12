#pragma once

#include "../servo_units/basilisk.h"

void SerialReplySender(Basilisk& b) {
  b.CommandBoth([](Servo& s) { s.Print(); });

  Serial.print("rho_l:");
  Serial.print(b.l_.GetReply().position);
  Serial.print(",");
  Serial.print("rho_r:");
  Serial.print(b.r_.GetReply().position);
  Serial.println();

  Serial.print("lpsx:");
  Serial.print(b.lps_.x_);
  Serial.print(",");
  Serial.print("lpsy:");
  Serial.print(b.lps_.y_);
  Serial.println();

  Serial.print("roll:");
  Serial.print(b.imu_.euler_[0]);
  Serial.print(",");
  Serial.print("pitch:");
  Serial.print(b.imu_.euler_[1]);
  Serial.print(",");
  Serial.print("yaw:");
  Serial.print(b.imu_.GetYaw(true));
  Serial.println();

  Serial.print("contact_l:");
  Serial.print(b.lego_.state_[0].contact, BIN);
  Serial.print(",");
  Serial.print("contact_r:");
  Serial.print(b.lego_.state_[1].contact, BIN);
  Serial.println();

  Serial.print("time_since_last_fix[0]:");
  Serial.print(b.mags_.TimeSinceLastFix(0), BIN);
  Serial.print(",");
  Serial.print("time_since_last_fix[1]:");
  Serial.print(b.mags_.TimeSinceLastFix(1), BIN);
  Serial.print(",");
  Serial.print("time_since_last_fix[2]:");
  Serial.print(b.mags_.TimeSinceLastFix(2), BIN);
  Serial.print(",");
  Serial.print("time_since_last_fix[3]:");
  Serial.print(b.mags_.TimeSinceLastFix(3), BIN);
  Serial.println();
}
