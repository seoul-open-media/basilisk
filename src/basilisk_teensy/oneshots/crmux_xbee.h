#pragma once

#include "meta.h"

void BasiliskOneshots::CRMuxXbee(Basilisk* b) {
  b->crmux_ = Basilisk::CRMux::Xbee;
}
