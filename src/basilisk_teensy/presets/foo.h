#pragma once

#include "meta.h"

void Presets::Foo(Basilisk* b) {
  auto& m = b->cmd_.mode;

  m = M::Idle_Init;
}
