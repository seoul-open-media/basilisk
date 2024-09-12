#include <beat.h>
#include <initializers.h>
#include <servo.h>
#include <specific/neokey3x4_i2c1.h>

#define CANFD_BUS 1

PmFmt pm_fmt{.maximum_torque = Res::kFloat,
             .velocity_limit = Res::kFloat,
             .accel_limit = Res::kFloat};

PmCmd pm_cmd_template{
    .maximum_torque = 32.0, .velocity_limit = 32.0, .accel_limit = 16.0};

QFmt q_fmt{[] {
  QFmt fmt;
  fmt.abs_position = Res::kFloat;
  fmt.motor_temperature = Res::kInt16;
  fmt.trajectory_complete = Res::kInt8;
  return fmt;
}()};

Servo servos[] = {
    {1, CANFD_BUS, &pm_fmt, &pm_cmd_template, PmCmdPosRelTo::Absolute, &q_fmt},
    {2, CANFD_BUS, &pm_fmt, &pm_cmd_template, PmCmdPosRelTo::Absolute, &q_fmt}};

template <typename ServoCommand>
void CommandAll(ServoCommand c) {
  for (Servo& s : servos) {
    c(s);
  }
}

uint16_t cmd;

Beat query_beat{10};
void Query() {
  CommandAll([](Servo& s) { s.Query(); });
}

Beat command_beat{10};
void Command() {
  CommandAll([](Servo& s) { s.Position(0.25 * cmd * (s.id_ % 2 ? 1 : -1)); });
}

auto& neokey = specific::neokey3x4_i2c1;
void neokey_cb(uint16_t key) {
  Serial.print(F("Key rose: "));
  Serial.println(key);
  cmd = key;
}
Beat receive_beat{10};
void Receive() { neokey.Run(); }

Beat print_beat{500};
void Print() {
  CommandAll([](Servo& s) { s.Print(); });
}

void setup() {
  SerialInitializer.init();
  I2C1Initializer.init();
  SpiInitializer.init();
  CanFdInitializer.init(CANFD_BUS);
  NeokeyInitializer.init(neokey);
  neokey.SetCommonRiseCallback(neokey_cb);

  CommandAll([](Servo& s) { s.Stop(); });
}

void loop() {
  if (receive_beat.Hit()) Receive();
  if (query_beat.Hit()) Query();
  if (command_beat.Hit()) Command();
  if (print_beat.Hit()) Print();
}
