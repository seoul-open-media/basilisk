#include <ACAN2517FD.h>
#include <Metro.h>
#include <Moteus.h>
#include <initializers.h>
#include <servo.h>

#define CANFD_BUS 1

class ServoUnit {
 public:
  ServoUnit() : servos{{1, CANFD_BUS}, {2, CANFD_BUS}} {}

  Servo servos[2];

  template <typename ServoCommand>
  void CommandAll(ServoCommand c) {
    c(&servos[0]);
    c(&servos[1]);
  }

  Metro metro{1000};
  void Executer() {
    uint16_t count = 0;
    while (1) {
      if (!metro.check()) {
        continue;
      }

      CommandAll([](Servo* s) { s->Query(); });
      double target = count % 2 ? 0.0 : 0.5;
      CommandAll([&](Servo* s) { s->Position(target); });
      count++;
    }
  }

  void Printer(const uint32_t& interval = 500) {
    Metro metro{interval};
    while (1) {
      if (metro.check()) {
        CommandAll([](Servo* s) { s->Print(); });
      }
    }
  }
} su;

void setup() {
  SerialInitializer.init();
  SpiInitializer.init();
  CanFdInitializer.init(CANFD_BUS);

  su.CommandAll([](Servo* servo) { servo->Stop(); });

  threads.addThread([] { su.Executer(); });
  threads.addThread([] { su.Printer(); });
}

void loop() {}
