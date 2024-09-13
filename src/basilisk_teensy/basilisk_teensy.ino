#include "cmd_rcvrs/neokey_cr.h"
#include "components/specifics/neokey3x4_i2c0.h"
#include "executer.h"
#include "helpers/imports.h"
#include "helpers/utils.h"
#include "rpl_sndrs/serial_rs.h"
#include "servo_units/basilisk.h"

// Basilisk and its executer.

struct Basilisk::Configuration cfg {
  .lps{.c = 300.0, .x_c = 300.0, .y_c = 300.0},
      .magnets{.pin_la = 3,
               .pin_lt = 4,
               .pin_ra = 5,
               .pin_rt = 6,
               .run_interval = 100},
      .lego {
    .pin_l = 23, .pin_r = 29, .run_interval = 20
  }
};

Basilisk* b = nullptr;
Executer* exec = nullptr;

// CommandReceivers.
Neokey nk = specifics::neokey3x4_i2c0;
NeokeyCommandReceiver nk_cr{nk};

void setup() {
  Serial.begin(9600);
  while (!Serial);

  b = new Basilisk{cfg};
  exec = new Executer{b};

  b->Setup();
  nk_cr.Setup(b);
}

void loop() {
  b->Run();

  static utils::Beat executer_beat{10};
  if (executer_beat.Hit()) exec->Run();

  static utils::Beat serial_plotter_rs_beat{100};
  if (serial_plotter_rs_beat.Hit()) SerialReplySender(*b);
}
