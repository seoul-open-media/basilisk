#pragma once

#include "meta.h"

void BasiliskOneshots::SetBaseYaw(Basilisk* b) {
  b->imu_.SetBaseYaw(b->cmd_.set_base_yaw.offset);
}
