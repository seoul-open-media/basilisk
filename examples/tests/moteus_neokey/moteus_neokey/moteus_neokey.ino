// For unknown reason, Querying and Commanding in the same thread freezes the
// thread, `threads.getState()` returning weird numbers.
// Make sure to Stop all Servos in `setup()` or the threads will get messy.

#include <ACAN2517FD.h>
#include <Adafruit_NeoKey_1x4.h>
#include <Metro.h>
#include <Moteus.h>
#include <Wire.h>
#include <initializers.h>
#include <neokey.h>
#include <servo.h>
#include <specific/neokey4x4_i2c1.h>

#define CANFD_BUS 1

void print_thread(String task_name) {
  Serial.print(task_name + " thread: ");
  Serial.print(threads.id());
  Serial.print(", ");
  Serial.println(threads.getState(threads.id()), HEX);
}

class NeokeyServoUnit {
 public:
  template <typename ServoCommand>
  void CommandUnit(ServoCommand c) {
    for (Servo& s : servos_) {
      c(s);
    }
  }

  struct Command {
    Threads::Mutex mutex;

    enum class Mode : uint8_t { Stop, DZero, SetPosition, Sine } mode;

    struct Stop {
      bool init;
    } stop;

    struct DZero {
      bool init;
    } d_zero;

    struct SetPosition {
      double position;
      enum class Progress : uint8_t {
        init,
        moving,
        complete,
        stopped
      } progress;
    } set_position;

    struct Sine {
      double amplitude = 0.25, frequency = 1.0, phase;
      enum class Progress : uint8_t {
        init,
        resuming,
        waving,
        stopped
      } progress;
    } sine;
  } cmd_;

  void QueryExecuter(const uint32_t& interval) {
    Metro metro{interval};
    while (1) {
      if (metro.check()) {
        print_thread("QueryExecuter");

        CommandUnit([](Servo& s) { s.Query(); });
      }

      yield();
    }
  }

  void CommandExecuter(const uint32_t& interval) {
    using M = Command::Mode;
    Metro metro{interval};
    while (1) {
      if (metro.check()) {
        print_thread("CommandExecuter");

        if (cmd_.mode == M::Stop) {
          ExecuteStop();
        } else if (cmd_.mode == M::SetPosition) {
          ExecuteSetPosition();
        }
      }

      yield();
    }
  }

  void ExecuteStop() {
    if (cmd_.stop.init) {
      Serial.println(F("ExecuteStop processing init state"));
      CommandUnit([](Servo& s) { s.Stop(); });
      cmd_.stop.init = false;
    }
  }

  // Using DiagnosticCommand during runtime might cause threads to freeze.
  // Setting base position and using base-relative commands seems to be
  // a better option.
  void ExecuteDZero() {
    if (cmd_.d_zero.init) {
      Serial.println(F("ExecuteDZero processing init state"));
      CommandUnit([&](Servo& s) {
        for (size_t i = 0; i < 8; i++) {
          // For unknown reason, commanding multiple times
          // has more stable result.
          s.d(F("d exact 0"));
          delay(10);
        }
        s.Position(0.0);
      });
      cmd_.d_zero.init = false;
    }
  }

  void ExecuteSetPosition() {
    using P = Command::SetPosition::Progress;
    switch (cmd_.set_position.progress) {
      case P::init: {
        Serial.println(F("ExecuteSetPosition processing init state"));
        CommandUnit([this](Servo& s) {
          s.Position(cmd_.set_position.position * (s.id_ % 2 ? 1 : -1));
        });
        cmd_.set_position.progress = P::moving;
      } break;
      case P::moving: {
        Serial.println(F("ExecuteSetPosition processing moving state"));
        for (Servo& s : servos_) {
          if (![&]() {
                // Refer to the WaitComplete example why we should Query twice.
                s.Query();
                s.Query();
                return s.GetReply().trajectory_complete;
              }()) {
            return;
          }
        }
        cmd_.set_position.progress = P::complete;
      } break;
      case P::complete: {
        Serial.println(F("ExecuteSetPosition processing complete state"));
        CommandUnit([](Servo& s) { s.Stop(); });
        cmd_.set_position.progress = P::stopped;
      } break;
      default:
        break;
    }
  }

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

  Servo servos_[2] = {{1, CANFD_BUS, &pm_fmt, &pm_cmd_template,
                       CommandPositionRelativeTo::Absolute, &q_fmt},
                      {2, CANFD_BUS, &pm_fmt, &pm_cmd_template,
                       CommandPositionRelativeTo::Absolute, &q_fmt}};
} neokey_su;

NeoKey1x4Callback NeokeyCallback(keyEvent evt) {
  if (evt.bit.EDGE == SEESAW_KEYPAD_EDGE_RISING) {
    auto key = evt.bit.NUM;

    Serial.print("Rise: ");
    Serial.println(key);

    using C = NeokeyServoUnit::Command;
    using M = C::Mode;
    auto& cmd = neokey_su.cmd_;
    auto& mode = cmd.mode;
    auto& mutex = neokey_su.cmd_.mutex;

    switch (key) {
      case 0: {
        Threads::Scope lock{mutex};
        mode = M::Stop;
        cmd.stop.init = true;
      } break;
      case 1: {
        Threads::Scope lock{mutex};
        mode = M::SetPosition;
        cmd.set_position.progress = C::SetPosition::Progress::init;
        cmd.set_position.position = 0;
      } break;
      case 2: {
        Threads::Scope lock{mutex};
        mode = M::SetPosition;
        cmd.set_position.progress = C::SetPosition::Progress::init;
        cmd.set_position.position = 0.5;
      } break;
      default: {
        Serial.println(F("Unknown command"));
      } break;
    }
  }

  return 0;
}

class NeokeyCommandReceiver {
 public:
  NeokeyCommandReceiver(Neokey& neokey) : neokey_{neokey} {}

  void Run(const uint32_t& interval) {
    Metro metro{interval};
    while (1) {
      if (metro.check()) {
        print_thread("NeokeyCommandReceiver");
        neokey_.read();
      }
      yield();
    }
  }

  Neokey& neokey_;
} neokey_cr{specific::neokey4x4_i2c1};

class SerialPrintReplySender {
 public:
  void Run(const uint32_t& interval) {
    Metro metro{interval};
    while (1) {
      if (metro.check()) {
        print_thread("SerialPrintReplySender");
        neokey_su.CommandUnit([](Servo& s) { s.Print(); });
      }
      yield();
    }
  }
} serial_print_rs;

void setup() {
  SerialInitializer.init();
  I2C0Initializer.init();
  if (CANFD_BUS == 1 || CANFD_BUS == 2) {
    SpiInitializer.init();
  } else if (CANFD_BUS == 3 || CANFD_BUS == 4) {
    Serial.println(F("Only Buses 1 and 2 work for T4_CanFd board v.1.5"));
    Spi1Initializer.init();
  } else {
    while (1) {
      Serial.println("Invalid CAN FD Bus");
      delay(1000);
    }
  }
  CanFdInitializer.init(CANFD_BUS);

  NeokeyInitializer.init(&neokey_cr.neokey_);
  neokey_cr.neokey_.registerCallbackAll(NeokeyCallback);

  neokey_su.CommandUnit([](Servo& s) { s.Stop(); });

  threads.addThread([] { neokey_su.CommandExecuter(1000); });
  threads.addThread([] { neokey_su.QueryExecuter(1000); });
  threads.addThread([] { neokey_cr.Run(1000); });
  threads.addThread([] { serial_print_rs.Run(1000); });
}

void loop() { yield(); }
