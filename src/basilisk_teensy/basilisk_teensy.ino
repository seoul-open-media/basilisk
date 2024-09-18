#include "cmd_rcvrs/neokey_cr.h"
#include "components/specifics/neokey3x4_i2c0.h"
#include "executer.h"
#include "helpers/imports.h"
#include "helpers/utils.h"
#include "rpl_sndrs/serial_rs.h"
#include "servo_units/basilisk.h"

// Basilisk configuration.
struct Basilisk::Configuration cfg {
  .servo{.id_l = 1, .id_r = 2, .bus = 1},
      .lps{.c = 300.0,
           .x_c = 300.0,
           .y_c = 300.0,
           .minx = 50.0,
           .maxx = 250.0,
           .miny = 50.0,
           .maxy = 250.0},
      .lego{.pin_l = 23, .pin_r = 29, .run_interval = 20},  //
      .mags {
    .pin_la = 3, .pin_lt = 4, .pin_ra = 5, .pin_rt = 6, .run_interval = 100
  }
};

// Basilisk and its executer.
Basilisk b{cfg};
Executer exec{&b};

// CommandReceivers.
Neokey nk = specifics::neokey3x4_i2c0;
NeokeyCommandReceiver nk_cr{nk};

void setup() {
  Serial.begin(9600);
  delay(1000);

  b.Setup();
  nk_cr.Setup(&b);
}

void loop() {
  b.Run();

  static Beat nk_cr_beat{10};
  if (nk_cr_beat.Hit()) nk_cr.Run();

  static Beat executer_beat{10};
  if (executer_beat.Hit()) exec.Run();

  static Beat serial_plotter_rs_beat{250};
  if (serial_plotter_rs_beat.Hit()) SerialReplySender(b);
}
