#include <ACAN2517FD.h>
#include <Adafruit_NeoKey_1x4.h>
#include <Metro.h>
#include <Moteus.h>
#include <Wire.h>

namespace mm = mjbots::moteus;
using PmCmd = mm::PositionMode::Command;
using PmFmt = mm::PositionMode::Format;
using QRpl = mm::Query::Result;
using QFmt = mm::Query::Format;
using Res = mm::Resolution;

#define MCP2518FD_CS 10
#define MCP2518FD_INT 41
ACAN2517FD canfd_driver(MCP2518FD_CS, SPI, MCP2518FD_CS);

class Initializer {};

class Basilisk {
 public:
  Basilisk()
      : pm_fmt_{.maximum_torque = Res::kFloat,
                .velocity_limit = Res::kFloat,
                .accel_limit = Res::kFloat},
        pm_cmd_template_{
            .maximum_torque = 32.0, .velocity_limit = 16.0, .accel_limit = 8.0},
        q_fmt_{[] {
          QFmt fmt;
          fmt.abs_position = Res::kFloat;
          fmt.motor_temperature = Res::kInt16;
          fmt.trajectory_complete = Res::kInt8;
          return fmt;
        }()},
        mot_l_{canfd_driver,
               [&] {
                 Moteus::Options options;
                 options.id = 1;
                 options.query_format = q_fmt_;
                 return options;
               }()},
        mot_r_{canfd_driver,
               [&] {
                 Moteus::Options options;
                 options.id = 2;
                 options.query_format = q_fmt_;
                 return options;
               }()},
        exec_metro_{10} {}

  template <typename MoteusMethod>
  void CommandLR(MoteusMethod f) {
    f(&mot_l_);
    f(&mot_r_);
  }

  void SetPositions(double pos_l, double pos_r) {
    mot_l_.SetPosition({.position = cmd_.set_positions.l});
    mot_r_.SetPosition({.position = cmd_.set_positions.r});
  }

  void execute() {
    if (!exec_metro_.check()) return;

    CommandLR([](Moteus* m) { m->SetQuery(); });

    switch (cmd_.mode) {
      case Command::Mode::Stop: {
        if (cmd_.stop.waiting) {
          CommandLR([](Moteus* m) { m->SetStop(); });
          cmd_.stop.waiting = false;
        }
        break;
      }
      case Command::Mode::Zero: {
        if (cmd_.zero.waiting) {
          CommandLR([](Moteus* m) {
            m->DiagnosticCommand(F("tel stop"));
            m->DiagnosticCommand(F("d exact 0"));
            m->SetPosition({.position = 0});
          });
          cmd_.zero.waiting = false;
        }
        break;
      }
      case Command::Mode::SetPositions: {
        switch (cmd_.set_positions.progress) {
          case Command::SetPosition::Progress::waiting: {
            SetPositions(cmd_.set_positions.l, cmd_.set_positions.r);
            cmd_.set_positions.progress =
                Command::SetPosition::Progress::moving;
            break;
          }
          case Command::SetPosition::Progress::moving: {
            if (mot_l_.last_result().values.trajectory_complete
                // && mot_r_.last_result().values.trajectory_complete
            ) {
              cmd_.set_positions.progress =
                  Command::SetPosition::Progress::complete;
            }
            break;
          }
          case Command::SetPosition::Progress::complete: {
            CommandLR([](Moteus* m) { m->SetStop(); });
            cmd_.set_positions.progress =
                Command::SetPosition::Progress::stopped;
            break;
          }
          default: {
            break;
          }
        }
        break;
      }
      case Command::Mode::Sine: {
        auto time_init = millis();
        SetPositions(1, 1);
        cmd_.sine.progress += millis() - time_init;
        break;
      }
      default: {
        break;
      }
    }
  }

  struct Command {
    enum class Mode : uint8_t {
      Stop,
      Zero,
      SetPositions,
      WalkForward,
      WalkBackward,
      GoToLpsPosition,
      Sine
    } mode;

    struct Stop {
      bool waiting;
    } stop;

    struct Zero {
      bool waiting;
    } zero;

    struct SetPosition {
      double l;
      double r;
      enum class Progress : uint8_t {
        waiting,
        moving,
        complete,
        stopped
      } progress;
    } set_positions;

    struct WalkForward {
      double speed;
    } walk_forward;

    struct WalkBackward {
      double speed;
    } walk_backward;

    struct GoToLpsPosition {
      double x, y;
      double speed;
    } go_to_lps_position;

    struct Sine {
      double amplitude;
      double frequency;
      uint32_t progress;
    } sine;
  } cmd_;

  Moteus mot_l_, mot_r_;

 private:
  const PmFmt pm_fmt_;
  const PmCmd pm_cmd_template_;
  const QFmt q_fmt_;
  Metro exec_metro_;
} basilisk;

class Executor {
} executor;

class NeokeyCommandReceiver {
 public:
  NeokeyCommandReceiver(uint16_t interval_ms) : neokey_{0x30, &Wire} {
    while (!neokey_.begin()) {
      Serial.println("Could not start NeoKey, check wiring?");
      delay(1000);
    }
    Serial.println(F("NeoKey started"));
  }

  void rcv() {
    if (!metro_.check()) return;

    const auto reading = neokey_.read();
    if (reading & (1 << 0)) {
      Serial.print("NeoKey #");
      Serial.print(0);
      Serial.println(" pressed. Basilisk CommandMode = Stop");
      basilisk.cmd_.mode = Basilisk::Command::Mode::Stop;
      basilisk.cmd_.stop.waiting = true;
    } else if (reading & (1 << 1)) {
      Serial.print("NeoKey #");
      Serial.print(1);
      Serial.println(" pressed. Basilisk CommandMode = Zero");
      basilisk.cmd_.mode = Basilisk::Command::Mode::Zero;
      basilisk.cmd_.zero.waiting = true;
    } else if (reading & (1 << 2)) {
      Serial.print("NeoKey #");
      Serial.print(2);
      Serial.println(" pressed. Basilisk CommandMode = SetPositions");
      basilisk.cmd_.mode = Basilisk::Command::Mode::SetPositions;
      basilisk.cmd_.set_positions.l = 0;
      basilisk.cmd_.set_positions.r = 0;
      basilisk.cmd_.set_positions.progress =
          Basilisk::Command::SetPosition::Progress::waiting;
    } else if (reading & (1 << 3)) {
      Serial.print("NeoKey #");
      Serial.print(3);
      Serial.println(" pressed. Basilisk CommandMode = Sine");
      basilisk.cmd_.mode = Basilisk::Command::Mode::Sine;
      basilisk.cmd_.sine.amplitude = 0.5;
      basilisk.cmd_.sine.frequency = 1;
    }
  }

 private:
  Adafruit_NeoKey_1x4 neokey_;
  Metro metro_{100};
} neokey_cmd_rcvr{5};

class SerialPrintReplySender {
 public:
  void snd() {
    if (!metro_.check()) return;

    print(basilisk.mot_l_.last_result().values);
    print(basilisk.mot_r_.last_result().values);
  }

 private:
  void print(Moteus::Query::Result reply) {
    Serial.print(F("time "));
    Serial.print(millis());
    Serial.print(F("  mode "));
    Serial.print(static_cast<int>(reply.mode));
    Serial.print(F("  pos "));
    Serial.print(reply.position);
    Serial.print(F("  vel "));
    Serial.print(reply.velocity);
    Serial.print(F("  fault "));
    Serial.print(reply.fault);
    Serial.println();
  }

  Metro metro_{250};
} serial_print_rpl_sndr;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    Serial.println(F("Serial problem"));
    delay(1000);
  }
  Serial.println(F("Serial started"));

  Wire.begin();
  Serial.println(F("I2C0 started"));

  SPI.begin();
  Serial.println(F("SPI started"));

  ACAN2517FDSettings settings(ACAN2517FDSettings::OSC_40MHz, 1000ll * 1000ll,
                              DataBitRateFactor::x1);
  settings.mArbitrationSJW = 2;
  settings.mDriverTransmitFIFOSize = 1;
  settings.mDriverReceiveFIFOSize = 2;
  const uint32_t errorCode =
      canfd_driver.begin(settings, [] { canfd_driver.isr(); });
  while (errorCode != 0) {
    Serial.print(F("CAN error 0x"));
    Serial.println(errorCode, HEX);
    delay(1000);
  }
  Serial.println(F("CAN FD started"));

  basilisk.mot_l_.SetStop();
  basilisk.mot_r_.SetStop();
}

void loop() {
  while (1) {
    basilisk.mot_l_.SetPosition({.position = 0});
  }

  neokey_cmd_rcvr.rcv();
  serial_print_rpl_sndr.snd();
  basilisk.execute();
}
