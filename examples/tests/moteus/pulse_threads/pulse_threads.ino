#include <ACAN2517FD.h>
#include <Metro.h>
#include <Moteus.h>
#include <TeensyThreads.h>
#include <initializers.h>

#define CANFD_BUS 1

Servo servos[] = {{1, CANFD_BUS}, {2, CANFD_BUS}};

template <typename ServoCommand>
void CommandAll(ServoCommand c) {
  for (Servo& s : servos) {
    c(s);
  }
}

void Query(const uint16_t interval) {
  Metro metro{interval};
  while (1) {
    if (metro.check()) {
      CommandAll([](Servo& s) { s.Query(); });
    }
  }
}

void Command(const uint16_t interval) {
  Metro metro{interval};
  static uint16_t count;
  while (1) {
    if (metro.check()) {
      double target = count % 2 ? 0.0 : 0.5;
      CommandAll([&](Servo& s) { s.Position(target); });
      count++;
    }
  }
}

void Print(const uint16_t interval) {
  Metro metro{interval};
  while (1) {
    if (metro.check()) {
      CommandAll([](Servo& s) { s.Print(); });
    }
  }
}

void setup() {
  SerialInitializer.init();
  SpiInitializer.init();
  CanFdInitializer.init(CANFD_BUS);

  CommandAll([](Servo& s) { s.Stop(); });

  threads.addThread([] { Query(10); });
  threads.addThread([] { Command(1000); });
  threads.addThread([] { Print(100); });
}

void loop() { yield(); }
