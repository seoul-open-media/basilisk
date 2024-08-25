// #include <beat.h>
// #include <initializers.h>

// #include "basilisk.h"
// #include "executer.h"
// #include "rpl_sndrs/serial_print_rs.h"
// #include "rpl_sndrs/serial_plotter_rs.h"

#include "components/specifics/neokey1x4_i2c0.h"
#include "helpers/helpers.h"
#include "servo_units/basilisk.h"

using namespace basilisk;

Basilisk b{1, 2};

Neokey& nk = specifics::neokey1x4_i2c0;

void setup() {
  initializers::serial.Init();
  initializers::spi0.Init();
  initializers::spi1.Init();
  initializers::canfd_driver.Init(1);
  initializers::i2c0.Init();
  initializers::i2c1.Init();
  initializers::neokey.Init(nk);

  // neokey_cr.Setup();
  // basilisk.ems_.SetPinMode();

  // basilisk.CommandBoth([](Servo& s) { s.Stop(); });
}

// Beat executer_beat{10};
// Beat neokey_cr_beat{25};
// Beat serial_print_rs_beat{500};
// Beat serial_plotter_rs_beat{50};

void loop() {
  // if (neokey_cr_beat.Hit()) neokey_cr.Run();
  // if (executer_beat.Hit()) executer.Run();
  // if (serial_print_rs_beat.Hit()) SerialPrintReplySender();
  // if (serial_plotter_rs_beat.Hit()) SerialPlotterReplySender();
}
