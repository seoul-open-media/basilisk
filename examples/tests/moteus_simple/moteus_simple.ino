#include <ACAN2517FD.h>
#include <Metro.h>
#include <Moteus.h>
#include <initializers.h>

#define CANFD_BUS 1

class ServoUnit {
 public:
  ServoUnit() : servos_{{1, CANFD_BUS}, {2, CANFD_BUS}} {}

  Servo servos_[2];

  void Query() {
    servos_[0].Query();
    servos_[1].Query();
  }

  void Executer() {
    static uint16_t count = 0;
    double target = count % 2 ? 0.0 : 0.5;
    servos_[0].Position(target);
    servos_[1].Position(target);
    count++;
  }

  void Printer() {
    servos_[0].Print();
    servos_[1].Print();
  }
} su;

void setup() {
  SerialInitializer.init();
  SpiInitializer.init();
  CanFdInitializer.init(CANFD_BUS);

  su.servos_[0].Stop();
  su.servos_[1].Stop();
}

Metro query{250};
Metro command{1000};
Metro print{500};

void loop() {
  if (query.check()) {
    su.Query();
  }

  if (command.check()) {
    su.Executer();
  }

  if (print.check()) {
    su.Printer();
  }
}
