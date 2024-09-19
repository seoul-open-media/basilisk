#pragma once

#include "../servo_units/basilisk.h"

struct Presets {
  using M = Basilisk::Command::Mode;

  enum Name : uint16_t {
    pFoo = 0,
  };

  static void Foo(Basilisk*);

  inline static const std::map<Name, void (*)(Basilisk*)> presets = {
      {pFoo, &Foo},
  };
};
