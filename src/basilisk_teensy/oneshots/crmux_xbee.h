#pragma once

#include "oneshots.h"

void BasiliskOneshots::CRMuxXbee(Basilisk* b) {
  b->crmux_ = Basilisk::CRMux::Xbee;
}
