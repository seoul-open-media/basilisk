#include <beat.h>
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

Beat query_beat{10};
void Query() {
  CommandAll([](Servo& s) { s.Query(); });
}

Beat command_beat{1000};
void Command() {
  static uint16_t count;
  double target = count % 2 ? 0.0 : 0.5;
  CommandAll([&](Servo& s) { s.Position(target); });
  count++;
}

Beat print_beat{500};
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
  if (query_beat.Hit()) Query();
  if (command_beat.Hit()) Command();
  if (print_beat.Hit()) Print();
}
