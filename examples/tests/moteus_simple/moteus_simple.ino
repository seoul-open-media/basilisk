#include <ACAN2517FD.h>
#include <Metro.h>
#include <Moteus.h>
#include <initializers.h>

#define CANFD_BUS 1

class ServoUnit {
 public:
  ServoUnit() : servos_{{1, CANFD_BUS}, {2, CANFD_BUS}} {}

  template <typename ServoCommand>
  void CommandAll(ServoCommand c) {
    for (Servo& s : servos_) {
      c(s);
    }
  }

  void Query() {
    CommandAll([](Servo& s) { s.Query(); });
  }

  void Command() {
    static uint16_t count;
    double target = count % 2 ? 0.0 : 0.5;
    CommandAll([&](Servo& s) { s.Position(target); });
    count++;
  }

  void Printer() {
    CommandAll([](Servo& s) { s.Print(); });
  }

  Servo servos_[2];
} su;

void setup() {
  SerialInitializer.init();
  SpiInitializer.init();
  CanFdInitializer.init(CANFD_BUS);

  su.CommandAll([](Servo& s) { s.Stop(); });
}

Metro query{10};
Metro command{1000};
Metro print{100};

void loop() {
  if (query.check()) {
    su.Query();
  }

  if (command.check()) {
    su.Command();
  }

  if (print.check()) {
    su.Printer();
  }
}
