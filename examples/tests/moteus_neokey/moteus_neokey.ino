#include <ACAN2517FD.h>
#include <Adafruit_NeoKey_1x4.h>
#include <Metro.h>
#include <Moteus.h>
#include <Wire.h>
#include <initializers.h>
#include <neokey.h>
#include <servo.h>

class NeokeyServoUnit {
 public:
  NeokeyServoUnit()
      : servos_{{1, &pm_fmt, &pm_cmd_template,
                 CommandPositionRelativeTo::Absolute, &q_fmt},
                {2, &pm_fmt, &pm_cmd_template,
                 CommandPositionRelativeTo::Absolute, &q_fmt}} {}

  struct Command {
    enum class Mode : uint8_t { Stop, DZero, SetPosition, Sine } mode;

    struct Values {
      struct Stop {
        bool waiting;
      } stop;

      struct DZero {
        bool waiting;
      } d_zero;

      struct SetPosition {
        double position;
        enum class Progress : uint8_t {
          waiting,
          moving,
          complete,
          stopped
        } progress;
      } set_position;

      struct Sine {
        double amplitude, frequency, phase;
        enum class Progress : uint8_t {
          waiting,
          going_to_resume_position,
          waving,
          stopped
        } progress;
      } sine;
    } values;
  } cmd_;

  template <typename ServoCommand>
  void CommandAll(ServoCommand c) {
    c(&servos_[0]);
    c(&servos_[1]);
  }

  void Executer(const uint32_t interval = 10) {
    Metro metro{interval};
    while (1) {
      if (metro.check()) {
        CommandAll([](Servo* servo) { servo->Query(); });

        (this->*executers_[(uint8_t)cmd_.mode])();
      }
    }
  }

  void ExecuteStop() { Serial.println("Stop"); }

  void ExecuteDZero() { Serial.println("DZero"); }

  void ExecuteSetPosition() { Serial.println("SetPosition"); }

  void ExecuteSine() { Serial.println("Sine"); }

  Servo servos_[2];
  void (NeokeyServoUnit::*executers_[4])() = {
      &ExecuteStop, &ExecuteDZero, &ExecuteSetPosition, &ExecuteSine};
  static inline PmFmt pm_fmt{.maximum_torque = Res::kFloat,
                             .velocity_limit = Res::kFloat,
                             .accel_limit = Res::kFloat};
  static inline PmCmd pm_cmd_template{
      .maximum_torque = 32.0, .velocity_limit = 16.0, .accel_limit = 4.0};
  static inline QFmt q_fmt{[] {
    QFmt fmt;
    fmt.abs_position = Res::kFloat;
    fmt.motor_temperature = Res::kInt16;
    fmt.trajectory_complete = Res::kInt8;
    return fmt;
  }()};
} neokey_su;

// Assume the following setup:
// I2C0        col 0
// row y         x    0 1 2 3  addr
//   0 0  _neokeys[0]{K K K K} 0x30

#define NEOKEY_DIM_X 4
#define NEOKEY_DIM_Y 1

Adafruit_NeoKey_1x4 neokey_mtx[NEOKEY_DIM_Y][NEOKEY_DIM_X / 4] = {
    Adafruit_NeoKey_1x4{0x30, &Wire}};

NeoKey1x4Callback NeokeyCallback(keyEvent evt) {
  if (evt.bit.EDGE == SEESAW_KEYPAD_EDGE_RISING) {
    auto key = evt.bit.NUM;
    Serial.print("Rise: ");
    Serial.println(key);

    neokey_su.cmd_.mode = (NeokeyServoUnit::Command::Mode)key;

    // neokey_su.cmd_.mode = NeokeyServoUnit::Command::Mode::SetPosition;
    // neokey_su.cmd_.values.set_position.position = 0.25 * key;
    // neokey_su.cmd_.values.set_position.progress =
    //     NeokeyServoUnit::Command::Values::SetPosition::Progress::waiting;
  }

  return 0;
}

class NeokeyCommandReceiver {
 public:
  NeokeyCommandReceiver(Adafruit_NeoKey_1x4* neokeys, uint8_t rows,
                        uint8_t cols)
      : neokey_{neokeys, rows, cols} {}

  void Run(uint32_t interval = 10) {
    Metro metro{interval};
    while (1) {
      if (metro.check()) {
        neokey_.read();
      }
    }
  }

  Neokey neokey_;
} neokey_cr{(Adafruit_NeoKey_1x4*)neokey_mtx, NEOKEY_DIM_Y, NEOKEY_DIM_X / 4};

class SerialPrintReplySender {
 public:
  template <typename ServoCommand>
  void CommandAll(ServoCommand c) {
    c(&servos_[0]);
    c(&servos_[1]);
  }

  void Run(const int32_t& interval = 250) {
    Metro metro_{interval};
    while (1) {
      if (metro_.check()) {
        CommandAll([](Servo* servo) { servo->Print(); });
      }
    }
  }

  Servo servos_[2] = {neokey_su.servos_[0], neokey_su.servos_[1]};
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

  neokey_su.CommandAll([](Servo* servo) { servo->SetStop(); });

  threads.addThread([] { neokey_su.Executer(); });
  threads.addThread([] { neokey_cr.Run(); });
  threads.addThread([] { serial_print_rs.Run(); });
}

void loop() {}
