#include <Metro.h>
#include <initializers.h>
#include <neokey.h>
#include <servo.h>
#include <specific/neokey3x4_i2c1.h>

#define CANFD_BUS 1

PmFmt pm_fmt{.maximum_torque = Res::kFloat,
             .velocity_limit = Res::kFloat,
             .accel_limit = Res::kFloat};

PmCmd pm_cmd_template{
    .maximum_torque = 32.0, .velocity_limit = 16.0, .accel_limit = 4.0};

QFmt q_fmt{[] {
  QFmt fmt;
  fmt.abs_position = Res::kFloat;
  fmt.motor_temperature = Res::kInt16;
  fmt.trajectory_complete = Res::kInt8;
  return fmt;
}()};

Servo servos[2] = {{1, CANFD_BUS, &pm_fmt, &pm_cmd_template,
                    CommandPositionRelativeTo::Absolute, &q_fmt},
                   {2, CANFD_BUS, &pm_fmt, &pm_cmd_template,
                    CommandPositionRelativeTo::Absolute, &q_fmt}};

uint16_t current_key;

template <typename ServoCommand>
void CommandAll(ServoCommand c) {
  for (Servo& s : servos) {
    c(s);
  }
}

void Query(const uint32_t interval) {
  Metro metro{interval};
  while (1) {
    if (metro.check()) {
      CommandAll([](Servo& s) { s.Query(); });
    }
  }
}

void Command(const uint32_t interval) {
  Metro metro{interval};
  while (1) {
    if (metro.check()) {
      CommandAll([](Servo& s) {
        s.Position(0.25 * current_key * (s.id_ % 2 ? 1 : -1));
      });
    }
  }
}

NeoKey1x4Callback neokey_cb(keyEvent evt) {
  if (evt.bit.EDGE == SEESAW_KEYPAD_EDGE_RISING) {
    current_key = evt.bit.NUM;
  }

  return 0;
}

auto& neokey = specific::neokey3x4_i2c1;

void Receive(const uint32_t interval) {
  Metro metro{interval};
  while (1) {
    if (metro.check()) {
      CommandAll([](Servo& s) { neokey.read(); });
    }
  }
}

void Print(const uint32_t interval) {
  Metro metro{interval};
  while (1) {
    if (metro.check()) {
      CommandAll([](Servo& s) { CommandAll([](Servo& s) { s.Print(); }); });
    }
  }
}

void setup() {
  SerialInitializer.init();
  I2C0Initializer.init();
  SpiInitializer.init();
  CanFdInitializer.init(CANFD_BUS);
  NeokeyInitializer.init(neokey);
  neokey.registerCallbackAll(neokey_cb);

  CommandAll([](Servo& s) { s.Stop(); });

  threads.addThread([] { Receive(10); });
  threads.addThread([] { Query(100); });
  threads.addThread([] { Command(10); });
  threads.addThread([] { Print(250); });
}

void loop() { yield(); }
