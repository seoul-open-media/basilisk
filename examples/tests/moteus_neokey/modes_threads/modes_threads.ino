#include <Metro.h>
#include <TeensyThreads.h>
#include <initializers.h>
#include <neokey.h>
#include <servo.h>
#include <specific/neokey3x4_i2c1.h>

void print_thread(String task_name) {
  Serial.print(task_name + " thread: ");
  Serial.print(threads.id());
  Serial.print(", ");
  Serial.println(threads.getState(threads.id()), HEX);
}

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

Servo servos[2] = {{1, CANFD_BUS, &pm_fmt, &pm_cmd_template,
                    CommandPositionRelativeTo::Absolute, &q_fmt},
                   {2, CANFD_BUS, &pm_fmt, &pm_cmd_template,
                    CommandPositionRelativeTo::Absolute, &q_fmt}};

template <typename ServoCommand>
void CommandAll(ServoCommand c) {
  for (Servo& s : servos) {
    c(s);
  }
}

// (A)
// struct Command {
//   double position;
// } cmd;

// (B)
double cmd;

void Query(const uint32_t interval) {
  Metro metro{interval};
  while (1) {
    if (metro.check()) {
      // print_thread("Query");
      CommandAll([](Servo& s) { s.Query(); });
    }
  }
}

void Command(const uint32_t interval) {
  Metro metro{interval};
  while (1) {
    if (metro.check()) {
      print_thread("Command");
      CommandAll([](Servo& s) { s.Position(cmd); });
    }
  }
}

NeoKey1x4Callback neokey_cb(keyEvent evt) {
  if (evt.bit.EDGE == SEESAW_KEYPAD_EDGE_RISING) {
    const auto key = evt.bit.NUM;
    Serial.print("Rise: ");
    Serial.println(key);
    cmd = 0.25 * evt.bit.NUM;
  }

  return 0;
}

auto& neokey = specific::neokey3x4_i2c1;

void Receive(const uint32_t interval) {
  Metro metro{interval};
  while (1) {
    if (metro.check()) {
      // print_thread("Receive");
      CommandAll([](Servo& s) { neokey.read(); });
    }
  }
}

void Print(const uint32_t interval) {
  Metro metro{interval};
  while (1) {
    if (metro.check()) {
      // print_thread("Print");
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

  threads.addThread([] { Receive(10); });  // ID 1 State 1(RUNNING)
  threads.addThread([] { Query(10); });    // ID 2 State 1(RUNNING)
  threads.addThread([] { Command(10); });  // ID 3 State 0x7FF80000
  threads.addThread([] { Print(50); });    // ID 4 State 1(RUNNING)
}

void loop() {
  for (int id = 1; id <= 4; id++) {
    Serial.print("Thread state: ");
    Serial.print(id);
    Serial.print(": ");
    Serial.println(threads.getState(id), HEX);
  }
  delay(500);
}
