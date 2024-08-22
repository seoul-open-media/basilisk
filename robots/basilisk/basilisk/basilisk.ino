#include <beat.h>
#include <initializers.h>

#include "basilisk.h"
#include "executer.h"
#include "rpl_sndrs/serial_print_rs.h"
#include "rpl_sndrs/serial_plotter_rs.h"

void setup() {
  SerialInitializer.init();
  SpiInitializer.init();
  Spi1Initializer.init();
  CanFdInitializer.init(CANFD_BUS);
  I2C0Initializer.init();
  I2C1Initializer.init();
  NeokeyInitializer.init(neokey);
  neokey_cr.Setup();
  basilisk.ems_.SetPinMode();

  basilisk.CommandBoth([](Servo& s) { s.Stop(); });
}

Beat executer_beat{10};
Beat neokey_cr_beat{25};
Beat serial_print_rs_beat{500};
Beat serial_plotter_rs_beat{50};

void loop() {
  if (neokey_cr_beat.Hit()) neokey_cr.Run();
  if (executer_beat.Hit()) executer.Run();
  if (serial_print_rs_beat.Hit()) SerialPrintReplySender();
  if (serial_plotter_rs_beat.Hit()) SerialPlotterReplySender();
}
