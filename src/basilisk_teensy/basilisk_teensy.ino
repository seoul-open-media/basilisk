#include "components/footswitches.h"
#include "components/imu.h"
#include "components/lps.h"
#include "helpers/imports.h"
#include "helpers/utils.h"

FootSwitches ftsw{23, 29};
Imu imu{};
Lps lps{300.0, 300.0, 300.0};
utils::Beat temp_beat{100};

void setup() {
  Serial.begin(9600);
  imu.Setup();
  lps.Setup();
  ftsw.Setup();
}

void loop() {
  // Serial.println("loop");
  {
    imu.Run();
    lps.Run();
  }

  if (temp_beat.Hit()) {
    ftsw.Poll();
    Serial.print("ftsw.accums_[0]:");
    Serial.print(ftsw.accums_[0]);
    Serial.print(",");
    Serial.print("ftsw.accums_[1]:");
    Serial.print(ftsw.accums_[1]);
    Serial.println();

    Serial.print("imu.euler_[0]:");
    Serial.print(imu.euler_[0]);
    Serial.print(",");
    Serial.print("imu.euler_[1]:");
    Serial.print(imu.euler_[1]);
    Serial.print(",");
    Serial.print("imu.euler_[2]:");
    Serial.print(imu.euler_[2]);
    Serial.print(",");
    Serial.print("imu.last_updated_time_:");
    Serial.print(imu.last_updated_time_);
    Serial.println();

    Serial.print("lps.dists_raw_[0]:");
    Serial.print(lps.dists_raw_[0]);
    Serial.print(",");
    Serial.print("lps.dists_raw_[1]:");
    Serial.print(lps.dists_raw_[1]);
    Serial.print(",");
    Serial.print("lps.dists_raw_[2]:");
    Serial.print(lps.dists_raw_[2]);
    Serial.print(",");
    Serial.print("lps.x_:");
    Serial.print(lps.x_);
    Serial.print(",");
    Serial.print("lps.y_:");
    Serial.print(lps.y_);
    Serial.println();
  }
}
