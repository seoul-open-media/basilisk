#include <Metro.h>
#include <TeensyThreads.h>
#include <initializers.h>
#include <neokey.h>
#include <servo.h>
#include <specific/neokey3x4_i2c1.h>

#include <atomic>

void print_thread(String task_name) {
  Serial.print(task_name + " thread: ");
  Serial.print(threads.id());
  Serial.print(", 0x");
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
//   uint8_t mode = 0x01;
//   double position;
// } cmd;

// (B)
std::atomic<uint16_t> cmd;

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
      uint16_t cmd_copy = cmd.load();
      if (cmd_copy < 4) {
        CommandAll([cmd_copy](Servo& s) { s.Position(cmd_copy); });
      } else {
        // Serial.println("w");  // OK
        // CommandAll([](Servo& s) { s.Position(0.0); });  // Fails!
      }

      // print_thread("Command");
      // if (cmd.mode == 0x00) {
      // CommandAll([](Servo& s) { s.Stop(); });
      // } else if (cmd.mode == 0x01) {

      // uint16_t cmd_copy = cmd.load();
      // if (cmd_copy < 4) {
      //   // CommandAll([cmd_copy](Servo& s) { s.Position(cmd_copy); });
      //   servos[0].Position(cmd_copy);
      // } else {
      //   // CommandAll([](Servo& s) { s.Stop(); });
      //   servos[1].Position(cmd_copy);
      // }

      // if (cmd_copy == 0x0000) {
      //   CommandAll([](Servo& s) { s.Stop(); });
      // }

      // if (cmd_copy) {
      // switch (cmd_copy) {
      //   case 0x0000:
      //     CommandAll([](Servo& s) { s.Stop(); });
      //     break;
      //   default:
      //     CommandAll([cmd_copy](Servo& s) { s.Position(0.25 * cmd_copy); });
      //     break;
      // }
      // } else {
      // CommandAll([](Servo& s) { s.Stop(); });
      // }
      // servos[0].Position(0.0);
      // }
    }
  }
}

NeoKey1x4Callback neokey_cb(keyEvent evt) {
  if (evt.bit.EDGE == SEESAW_KEYPAD_EDGE_RISING) {
    // const auto key = evt.bit.NUM;
    // Serial.print("Rise: ");
    // Serial.println(key);
    // if (key < 4) {
    // cmd.mode = 0x00;
    // } else {
    // cmd.mode = 0x01;
    cmd = evt.bit.NUM;
    // }
  }

  return 0;
}

auto& neokey = specific::neokey3x4_i2c1;

void Receive(const uint32_t interval) {
  Metro metro{interval};
  while (1) {
    if (metro.check()) {
      // print_thread("Receive");
      neokey.read();
    }
  }
}

void Print(const uint32_t interval) {
  Metro metro{interval};
  while (1) {
    if (metro.check()) {
      // print_thread("Print");
      CommandAll([](Servo& s) { s.Print(); });
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

  Serial.println("Boot. Wait 3s...");
  delay(3000);

  threads.addThread([] { Receive(10); });  // ID 1 State 1(RUNNING)
  // delay(1);
  threads.addThread([] { Query(10); });  // ID 2 State 1(RUNNING)
  // delay(1);
  threads.addThread([] { Command(10); });  // ID 3 State 0x7FF80000
  // delay(1);
  threads.addThread([] { Print(250); });  // ID 4 State 1(RUNNING)
  // delay(1);
}

Metro thread_print_metro{1000};

void loop() {
  if (thread_print_metro.check()) {
    for (int id = 1; id <= 4; id++) {
      Serial.print("Thread state: ");
      Serial.print(id);
      Serial.print(": 0x");
      Serial.println(threads.getState(id), HEX);
    }
  }

  // if (threads.getState(3) != Threads::RUNNING) {
  //   threads.kill(3);
  //   threads.addThread([] { Command(10); });
  // }

  yield();
}
