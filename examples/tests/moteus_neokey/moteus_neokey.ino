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

Servo servos_bus1[]{
    {1, &pm_fmt, &pm_cmd_template, CommandPositionRelativeTo::Absolute, &q_fmt},
    {2, &pm_fmt, &pm_cmd_template, CommandPositionRelativeTo::Absolute,
     &q_fmt}};

// template <typename ServoCommand>
// void CommandBus1(ServoCommand c) {
//   for (size_t i = 0; i < sizeof(servos_bus1) / sizeof(servos_bus1[0]); i++) {
//     c(servos_bus1 + i);
//   }
// }

class NeokeyServoUnit {
 public:
  NeokeyServoUnit() : servos_{servos_bus1}, num_servos_{2} {}

  template <typename ServoCommand>
  void CommandUnit(ServoCommand c) {
    for (size_t i = 0; i < num_servos_; i++) {
      c(servos_ + i);
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
      double amplitude = 0.5, frequency = 2.0, phase;
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
        Serial.print(F("QueryExecuter thread: "));
        Serial.print(threads.id());
        Serial.print(", ");
        Serial.println(threads.getState(threads.id()), HEX);

        CommandUnit([](Servo* s) { s->Query(); });
      }
      delay(interval >> 2);
    }
  }

  void CommandExecuter(const uint32_t& interval) {
    using M = Command::Mode;
    Metro metro{interval};
    while (1) {
      if (metro.check()) {
        Serial.print(F("CommandExecuter thread: "));
        Serial.print(threads.id());
        Serial.print(", ");
        Serial.println(threads.getState(threads.id()), HEX);

        if (cmd_.mode == M::Stop) {
          ExecuteStop();
        } else if (cmd_.mode == M::SetPosition) {
          ExecuteSetPosition();
        }
      }
      delay(interval >> 2);
    }
  }

  void ExecuteStop() {
    if (cmd_.stop.init) {
      Serial.println(F("ExecuteStop processing init state"));
      CommandUnit([](Servo* s) { s->Stop(); });
      cmd_.stop.init = false;
    }
  }

  // Using DiagnosticCommand during runtime might cause threads to freeze.
  // Setting base position and using base-relative commands seems to be
  // a better option.
  void ExecuteDZero() {
    if (cmd_.d_zero.init) {
      Serial.println(F("ExecuteDZero processing init state"));
      CommandUnit([&](Servo* s) {
        s->Stop();
        s->d(F("tel stop"));
        s->SetDiagnosticFlush();
        for (size_t i = 0; i < 8; i++) {
          // For unknown reason, commanding multiple times
          // has more stable result.
          s->d(F("d exact 0"));
          delay(10);
        }
        s->Position(0);
      });
      cmd_.d_zero.init = false;
    }
  }

  void ExecuteSetPosition() {
    using P = Command::SetPosition::Progress;
    switch (cmd_.set_position.progress) {
      case P::init: {
        Serial.println(F("ExecuteSetPosition processing init state"));
        CommandUnit([this](Servo* s) {
          s->Position(cmd_.set_position.position * (s->id_ % 2 ? 1 : -1));
        });
        cmd_.set_position.progress = P::moving;
      } break;
      case P::moving: {
        Serial.println(F("ExecuteSetPosition processing moving state"));
        for (size_t i = 0; i < num_servos_; i++) {
          if (![&]() {
                // Refer to the WaitComplete example why we should Query twice.
                servos_[i].SetQuery();
                servos_[i].SetQuery();
                return servos_[i].last_result().values.trajectory_complete;
              }()) {
            return;
          }
        }
        cmd_.set_position.progress = P::complete;
      } break;
      case P::complete: {
        Serial.println(F("ExecuteSetPosition processing complete state"));
        CommandUnit([](Servo* servo) { servo->Stop(); });
        cmd_.set_position.progress = P::stopped;
      } break;
      default:
        break;
    }
  }

  Servo* servos_;
  const uint8_t num_servos_;
} neokey_su;

#define NEOKEY_DIM_X 4
#define NEOKEY_DIM_Y 1

Adafruit_NeoKey_1x4 neokey_mtx[NEOKEY_DIM_Y][NEOKEY_DIM_X / 4] = {
    Adafruit_NeoKey_1x4{0x30, &Wire}};

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
  NeokeyCommandReceiver(Adafruit_NeoKey_1x4* neokeys, uint8_t rows,
                        uint8_t cols)
      : neokey_{neokeys, rows, cols} {}

  void Run(const uint32_t& interval = 25) {
    Metro metro{interval};
    while (1) {
      if (metro.check()) {
        neokey_.read();
      }
      delay(interval >> 2);
    }
  }

  Neokey neokey_;
} neokey_cr{(Adafruit_NeoKey_1x4*)neokey_mtx, NEOKEY_DIM_Y, NEOKEY_DIM_X / 4};

class SerialPrintReplySender {
 public:
  void Run(const uint32_t& interval = 250) {
    Metro metro{interval};
    while (1) {
      if (metro.check()) {
        // CommandAll([](Servo* servo) { servo->Print(); });
        servos_bus1[0].Print();
      }
    }
  }
} serial_print_rs;

void setup() {
  SerialInitializer.init();
  I2C0Initializer.init();
  SpiInitializer.init();
  CanFdInitializer.init();

  neokey_cr.neokey_.begin();
  for (uint8_t y = 0; y < NEOKEY_DIM_Y; y++) {
    for (uint8_t x = 0; x < NEOKEY_DIM_X; x++) {
      neokey_cr.neokey_.registerCallback(x, y, NeokeyCallback);
    }
  }

  servos_bus1[0].Stop();
  servos_bus1[1].Stop();
  // CommandBus1([](Servo* s) { s->Stop(); });  // Why not working?

  threads.addThread([] { neokey_su.CommandExecuter(100); });
  threads.addThread([] { neokey_su.QueryExecuter(100); });
  threads.addThread([] { neokey_cr.Run(); });
  // threads.addThread([] { serial_print_rs.Run(); });
}

void loop() {
  Serial.println("Loop is alive");
  delay(2000);
}
