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

Beat query_beat{100};
void Query() {
  CommandAll([](Servo& s) { s.Query(); });
}

Beat command_beat{10};
void Command() {
  CommandAll([](Servo& s) { s.Position(0.25 * sin(millis() / 250.0)); });
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
