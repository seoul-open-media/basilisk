#pragma once

#include "../helpers/imports.h"

#define IMU_SERIAL (Serial2)

/* Angle unit of incoming data from the EBIMU board is 'degrees' between
 * -180.0 and 180.0, but the rest of the program assumes 'revolutions'
 * as angle unit for compatibility with moteus servomotor controllers.
 * The field `double euler_[3]` saves angles in revolutions, between
 * -0.5 and 0.5. */
class Imu {
 public:
  Imu() : euler_{0.0} {}

  // Must be called before use.
  void Setup() {
    if (!setup_cplt_) {
      IMU_SERIAL.begin(57600);
      setup_cplt_ = true;
    }
  }

  // Should be called continuously to immediately receive to
  // incoming sensor data and prevent Serial buffer overflow.
  void Run() {
    if (!setup_cplt_) {
      Serial.println("IMU: Setup NOT complete");
      return;
    }

    static const int buf_size = 64;
    static char buf[buf_size];
    static int buf_idx = 0;
    static const auto increment_idx = [&] {
      if (++buf_idx >= 64) buf_idx = 0;
    };

    const auto rbytes = IMU_SERIAL.available();
    for (int i = 0; i < rbytes; i++, increment_idx()) {
      buf[buf_idx] = IMU_SERIAL.read();
      if (buf[buf_idx] == '\n') {
        char* temp[3];
        temp[0] = strtok(buf, ",");
        temp[1] = strtok(nullptr, ",");
        temp[2] = strtok(nullptr, ",");
        if (![&] {
              for (const auto* s : temp) {
                if (!s) return false;
              }
              return true;
            }()) {
          continue;
        }
        for (int i = 0; i < 3; i++) {
          euler_[i] = atof(temp[i]) / 360.0;
        }
        last_updated_time_ = millis();
        buf_idx = -1;
      } else if (buf[buf_idx] == '*') {
        buf_idx = -1;
      }
    }
  }

  void SetBaseYaw() { base_yaw_ = euler_[3]; }

  double GetYawRelToBase() { return euler_[3] - base_yaw_; }

  double euler_[3];  // [0]: roll, [1]: pitch, [2]: yaw, in revs
  double base_yaw_ = 0.0;
  uint32_t last_updated_time_;

 private:
  bool setup_cplt_ = false;
};
