#include <ACAN2517FD.h>
#include <Metro.h>
#include <Moteus.h>
#include <TeensyThreads.h>
#include <initializers.h>

#define CANFD_BUS 1

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
    Metro metro{10};
    while (1) {
      if (metro.check()) {
        CommandUnit([](Servo& s) { s.Query(); });
      }
    }
  }

  void Command() {
    static uint16_t count;
    Metro metro{1000};
    while (1) {
      if (metro.check()) {
        double target = count % 2 ? 0.0 : 0.5;
        CommandUnit([&](Servo& s) { s.Position(target); });
        count++;
      }
    }
  }

  void Print() {
    Metro metro{100};
    while (1) {
      if (metro.check()) {
        CommandUnit([](Servo& s) { s.Print(); });
      }
    }
  }

  Servo servos_[2];
} su;

void setup() {
  SerialInitializer.init();
  SpiInitializer.init();
  CanFdInitializer.init(CANFD_BUS);

  su.CommandUnit([](Servo& s) { s.Stop(); });

  threads.addThread([] { su.Query(); });
  threads.addThread([] { su.Command(); });
  threads.addThread([] { su.Print(); });
}

void loop() {}
