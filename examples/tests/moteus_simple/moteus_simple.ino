#include <ACAN2517FD.h>
#include <Metro.h>
#include <Moteus.h>
#include <initializers.h>
#include <servo.h>

#define CANFD_BUS 1

Servo servos[] = {{1, CANFD_BUS}, {2, CANFD_BUS}};
template <typename ServoCommand>
void CommandAll(ServoCommand c) {
  for (uint8_t i = 0; i < 2; i++) {
    c(&servos[i]);
  }
}

void Executer() {
  Metro metro{1000};
  uint16_t count = 0;
  while (1) {
    if (metro.check()) {
      CommandAll([](Servo* s) { s->Query(); });
      double target = count % 2 ? 0.0 : 0.5;
      CommandAll([&](Servo* s) { s->Position(target); });
      count++;
    }
  }
}

void Printer() {
  Metro metro{1000};
  while (1) {
    if (metro.check()) {
      CommandAll([](Servo* s) { s->Print(); });
    }
  }
}

void setup() {
  SerialInitializer.init();
  SpiInitializer.init();
  CanFdInitializer.init(CANFD_BUS);

  CommandAll([](Servo* servo) { servo->Stop(); });

  threads.addThread([] { Executer(); });
  threads.addThread([] { Printer(); });
}

void loop() {}
