#include <ACAN2517FD.h>
#include <Adafruit_NeoKey_1x4.h>
#include <Metro.h>
#include <Moteus.h>
#include <Wire.h>
#include <initializers.h>
#include <neokey.h>
#include <servo.h>

Servo temp_servo{2};

class NeokeyServoUnit {
 public:
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

  void Executer(const uint32_t& interval) {
    Metro metro{interval};
    while (1) {
      // Serial.println("while begin");
      if (metro.check()) {
        Serial.println("metro check");
        Serial.print(threads.id());
        Serial.print(", ");
        Serial.println(threads.getState(threads.id()), HEX);

        if (cmd_.mode == Command::Mode::Stop) {
          ExecuteStop();
        } 
        else if (cmd_.mode == Command::Mode::DZero) {
          ExecuteDZero();
        } 
        else if (cmd_.mode == Command::Mode::SetPosition) {
          ExecuteSetPosition();
        }

        Serial.print(threads.id());
        Serial.print(", ");
        Serial.println(threads.getState(threads.id()), HEX);
      }

      // Serial.println("while end1");
      // Serial.println("while end2");
      // Serial.println("while end3");
      // Serial.println("while end4");
    }
  }

  void ExecuteStop() {
    Serial.println(cmd_.stop.init ? "init == true" : "init==false");
    if (cmd_.stop.init) {
      temp_servo.Stop();
      cmd_.stop.init = false;
    }

    // if (cmd_.stop.init) {
    //   Serial.println("init");
    //   // CommandAll([](Servo* servo) { servo->Stop(); });
    //   servos_[1].Stop();
    //   Serial.println("init processed");
    //   cmd_.stop.init = false;
    //   Serial.println("set stop.init = false");
    // }
  }

  void ExecuteDZero() {
    if (cmd_.d_zero.init) {
      Serial.println("ExecuteDZero working");
      temp_servo.Stop();
      temp_servo.d(F("tel stop"));
      temp_servo.d(F("d exact 0"));
      // CommandAll([](Servo* servo) {
      //   servo->Stop();
      //   servo->DiagnosticCommand(F("tel stop"));
      //   servo->DiagnosticCommand(F("d exact 0"));
      //   // servo->Position(0);
      // });
      cmd_.d_zero.init = false;
    }
  }

  void ExecuteSetPosition() {
    Serial.print(threads.id());
    Serial.print(", ");
    Serial.println(threads.getState(threads.id()), HEX);
    Serial.println("ExecuteSetPosition");
    if (cmd_.set_position.progress == Command::SetPosition::Progress::init) {
      Serial.println("ExecuteSetPosition Actual work");
      Serial.print(threads.id());
      Serial.print(", ");
      Serial.println(threads.getState(threads.id()), HEX);

      temp_servo.Position(0);

      Serial.println("ExecuteSetPosition Actual work DONE");
      Serial.print(threads.id());
      Serial.print(", ");
      Serial.println(threads.getState(threads.id()), HEX);

      cmd_.set_position.progress = Command::SetPosition::Progress::moving;
    }
    // temp_servo.cmd_.set_position.progress) {
    //   temp_ser. Command::SetPosition::Progress::init: {
    //     CommandAll([&](Servo* servo) {
    //       servo->Position(cmd_.set_position.position *
    //                       (servo->id_ % 2 ? 1 : -1));
    //     });
    //     cmd_.set_position.progress = Command::SetPosition::Progress::moving;
    //   } break;
    //   case Command::SetPosition::Progress::moving: {
    //     for (size_t i = 0; i < sizeof(servos_) / sizeof(servos_[0]); i++) {
    //       if (!servos_[i].last_result().values.trajectory_complete) {
    //         return;
    //       }
    //     }
    //     cmd_.set_position.progress =
    //     Command::SetPosition::Progress::complete;
    //   } break;
    //   case Command::SetPosition::Progress::complete: {
    //     CommandAll([](Servo* servo) { servo->Stop(); });
    //     cmd_.set_position.progress = Command::SetPosition::Progress::stopped;
    //   } break;
    //   default:
    //     break;
    // }
  }
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
    // auto& mutex = neokey_su.cmd_.mutex;

    mode = (M)key;
    switch (mode) {
      case M::Stop: {
        Threads::Scope lock{neokey_su.cmd_.mutex};
        Serial.println("key locked mutex");
        cmd.stop.init = true;
        Serial.println("set stop.init = true");
      }
        Serial.println("key UNlocked mutex");
        break;
      case M::DZero: {
        Threads::Scope lock{neokey_su.cmd_.mutex};
        cmd.d_zero.init = true;
      } break;
      case M::SetPosition: {
        Threads::Scope lock{neokey_su.cmd_.mutex};
        cmd.set_position.progress = C::SetPosition::Progress::init;
        cmd.set_position.position = 0;
      } break;
      case M::Sine: {
        Threads::Scope lock{neokey_su.cmd_.mutex};
        cmd.sine.progress = C::Sine::Progress::init;
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
    }
  }

  Neokey neokey_;
} neokey_cr{(Adafruit_NeoKey_1x4*)neokey_mtx, NEOKEY_DIM_Y, NEOKEY_DIM_X / 4};

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

  // neokey_su.CommandAll([](Servo* servo) { servo->Stop(); });

  // temp_servo.Position(0);
  temp_servo.Stop();

  threads.addThread([] { neokey_su.Executer(500); });
  threads.addThread([] { neokey_cr.Run(); });
  // threads.addThread([] { serial_print_rs.Run(); });
}

void loop() {
  Serial.println("loop");
  delay(250);
}
