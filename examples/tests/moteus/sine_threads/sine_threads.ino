#include <ACAN2517FD.h>
#include <Metro.h>
#include <Moteus.h>
#include <TeensyThreads.h>
#include <initializers.h>
#include <servo.h>

#define CANFD_BUS 1

Servo servos[] = {{1, CANFD_BUS}, {2, CANFD_BUS}};

template <typename ServoCommand>
void CommandAll(ServoCommand c) {
  for (Servo& s : servos) {
    c(s);
  }
}

void Query(const uint32_t& interval) {
  Metro metro{interval};
  while (1) {
    if (metro.check()) {
      CommandAll([](Servo& s) { s.Query(); });
    }
  }
}

void Command(const uint32_t& interval) {
  Metro metro{interval};
  while (1) {
    if (metro.check()) {
      CommandAll([](Servo& s) { s.Position(0.25 * sin(millis() / 250.0)); });
    }
  }
}

void Print(const uint32_t& interval) {
  Metro metro{interval};
  while (1) {
    if (metro.check()) {
      CommandAll([](Servo& servo) { servo.Print(); });
    }
  }
}

void setup() {
  SerialInitializer.init();
  SpiInitializer.init();
  CanFdInitializer.init(CANFD_BUS);

  CommandAll([](Servo& s) { s.Stop(); });

  threads.addThread([] { Query(100); });
  threads.addThread([] { Command(10); });
  threads.addThread([] { Print(100); });
}

void loop() { yield(); }
