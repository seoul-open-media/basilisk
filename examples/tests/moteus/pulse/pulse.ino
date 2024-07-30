#include <Metro.h>
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

Metro query_metro{10};
void Query() {
  CommandAll([](Servo& s) { s.Query(); });
}

Metro command_metro{1000};
void Command() {
  static uint16_t count;
  double target = count % 2 ? 0.0 : 0.5;
  CommandAll([&](Servo& s) { s.Position(target); });
  count++;
}

Metro print_metro{100};
void Print() {
  CommandAll([](Servo& s) { s.Print(); });
}

void setup() {
  SerialInitializer.init();
  SpiInitializer.init();
  CanFdInitializer.init(CANFD_BUS);

  CommandAll([](Servo& s) { s.Stop(); });
}

void loop() {
  if (query_metro.check()) Query();
  if (command_metro.check()) Command();
  if (print_metro.check()) Print();
}
