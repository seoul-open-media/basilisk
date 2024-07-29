#include <ACAN2517FD.h>
#include <Metro.h>
#include <Moteus.h>
#include <TeensyThreads.h>
#include <initializers.h>

#define CANFD_BUS 1

Metro query{10};
Metro command{10};
Metro print{100};

class ServoUnit {
 public:
  ServoUnit() : servos_{{1, CANFD_BUS}, {2, CANFD_BUS}} {}

  template <typename ServoCommand>
  void CommandUnit(ServoCommand c) {
    for (Servo& s : servos_) {
      c(s);
    }
  }

  void Query() {
    CommandUnit([](Servo& s) { s.Query(); });
  }

  void Command() {
    static uint16_t count;
    double target = count % 2 ? 0.0 : 0.5;
    CommandUnit([&](Servo& s) { s.Position(target); });
    count++;
  }

  void Print() {
    CommandUnit([](Servo& s) { s.Print(); });
  }

  Servo servos_[2];
} su;

void setup() {
  SerialInitializer.init();
  SpiInitializer.init();
  CanFdInitializer.init(CANFD_BUS);

  su.CommandUnit([](Servo& s) { s.Stop(); });

  threads.addThread([] {
    while (1) {
      if (query.check()) {
        su.Query();
      }
    }
  });

  threads.addThread([] {
    while (1) {
      if (command.check()) {
        su.Command();
      }
    }
  });

  threads.addThread([] {
    while (1) {
      if (print.check()) {
        su.Print();
      }
    }
  });
}

void loop() {}
