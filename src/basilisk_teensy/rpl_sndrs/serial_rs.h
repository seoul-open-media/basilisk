#pragma once

#include "../servo_units/basilisk.h"

void SerialReplySender(Basilisk& b) {
  // Servo Replies
  b.CommandBoth([](Servo* s) { s->Print(); });

  // Servo outputs
  Serial.print("phi_l:");
  Serial.print(b.l_.GetReply().abs_position, 4);
  Serial.print(",");
  Serial.print("phi_r:");
  Serial.print(b.r_.GetReply().abs_position, 4);
  Serial.println();

  // LPS position
  // Serial.print("lpsx=");
  // Serial.print(b.lps_.x_);
  // Serial.print(";");
  // Serial.print("lpsy=");
  // Serial.print(b.lps_.y_);
  // Serial.println();

  // LPS debug
  // Serial.print("b.lps_.dists_raw_[0]:");
  // Serial.print(b.lps_.dists_raw_[0]);
  // Serial.print(",");
  // Serial.print("b.lps_.dists_raw_[1]:");
  // Serial.print(b.lps_.dists_raw_[1]);
  // Serial.print(",");
  // Serial.print("b.lps_.dists_raw_[2]:");
  // Serial.print(b.lps_.dists_raw_[2]);
  // Serial.println();

  // Serial.print("b.lps_.error_.bytes[0]:");
  // Serial.print(b.lps_.error_.bytes[0]);
  // Serial.print(",");
  // Serial.print("b.lps_.error_.bytes[1]:");
  // Serial.print(b.lps_.error_.bytes[1]);
  // Serial.print(",");
  // Serial.print("b.lps_.error_.bytes[2]:");
  // Serial.print(b.lps_.error_.bytes[2]);
  // Serial.println();

  // IMU orientation
  // Serial.print("roll:");
  // Serial.print(b.imu_.euler_[0], 3);
  // Serial.print(",");
  // Serial.print("pitch:");
  // Serial.print(b.imu_.euler_[1], 3);
  // Serial.print(",");
  Serial.print("yaw:");
  Serial.print(b.imu_.GetYaw(true), 4);
  Serial.println();

  // Lego
  // Serial.print("contact_l=");
  // Serial.print(b.lego_.state_[0].contact, BIN);
  // Serial.print("/");
  // Serial.print("contact_r=");
  // Serial.print(b.lego_.state_[1].contact, BIN);
  // Serial.println();

  // Magnets
  // Serial.print("time_since_last_attach[0]=");
  // Serial.print(b.mags_.time_since_last_attach_[0]);
  // Serial.print("/");
  // Serial.print("time_since_last_attach[1]=");
  // Serial.print(b.mags_.time_since_last_attach_[1]);
  // Serial.print("/");
  // Serial.print("time_since_last_attach[2]=");
  // Serial.print(b.mags_.time_since_last_attach_[2]);
  // Serial.print("/");
  // Serial.print("time_since_last_attach[3]=");
  // Serial.print(b.mags_.time_since_last_attach_[3]);
  // Serial.println();
}
