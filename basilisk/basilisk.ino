#include <ACAN2517FD.h>
#include <Adafruit_NeoKey_1x4.h>
#include <Metro.h>
#include <Moteus.h>
#include <Wire.h>

#define MCP2517_CS 10
#define MCP2517_INT 41
ACAN2517FD can(MCP2517_CS, SPI, MCP2517_INT);

class Basilisk {
 public:
  Basilisk()
      : mot_l_{can,
               [&] {
                 Moteus::Options options;
                 options.id = 1;
                 options.query_format = rpl_fmt_;
                 return options;
               }()},
        mot_r_{can, [&] {
                 Moteus::Options options;
                 options.id = 2;
                 options.query_format = rpl_fmt_;
                 return options;
               }()} {}

  template <typename MoteusMethod>
  void CommandLR(MoteusMethod f) {
    f(&mot_l_);
    f(&mot_r_);
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
        if (cmd_.stop.waiting) {
          CommandLR([](Moteus* m) {
            m->DiagnosticCommand(F("tel stop"));
            Serial.println(m->DiagnosticCommand(F("d exact 0")));
          });
        }
        break;
      }
      case Command::Mode::SetPosition: {
        switch (cmd_.set_position.progress) {
          case Command::SetPosition::Progress::waiting: {
            SetPosition(cmd_.set_position.position_l,
                        cmd_.set_position.position_r);
            cmd_.set_position.progress =
                Command::SetPosition::Progress::processing;
            break;
          }
          case Command::SetPosition::Progress::processing: {
            if (mot_l_.last_result().values.trajectory_complete &&
                mot_r_.last_result().values.trajectory_complete) {
              cmd_.set_position.progress =
                  Command::SetPosition::Progress::complete;
            }
            break;
          }
          case Command::SetPosition::Progress::complete: {
            Serial.println("SetPosition complete");
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

        SetPosition(

            1, 1

        );

        cmd_.sine.progress += millis() - time_init;
        break;
      }
      default: {
        break;
      }
    }
  }

  void SetPosition(double pos_l, double pos_r) {
    mot_l_.SetPosition(Moteus::PositionMode::Command{
        .position = cmd_.set_position.position_l});
    mot_r_.SetPosition(Moteus::PositionMode::Command{
        .position = cmd_.set_position.position_r});
  }

  struct Command {
    enum class Mode : uint8_t {
      Stop,
      Zero,
      SetPosition,
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
      double position_l;
      double position_r;
      enum class Progress : uint8_t { waiting, processing, complete } progress;
    } set_position;

    struct WalkForward {
      double velocity;
    } walk_forward;

    struct WalkBackward {
      double velocity;
    } walk_backward;

    struct GoToLpsPosition {
      double x, y;
    } go_to_lps_position;

    struct Sine {
      double amplitude;
      double frequency;
      uint32_t progress;
    } sine;
  } cmd_;

  Moteus mot_l_, mot_r_;

 private:
  const Moteus::PositionMode::Format cmd_fmt_{
      .velocity = mjbots::moteus::Resolution::kIgnore,
      .maximum_torque = mjbots::moteus::Resolution::kFloat,
      .velocity_limit = mjbots::moteus::Resolution::kFloat,
      .accel_limit = mjbots::moteus::Resolution::kFloat};

  const Moteus::Query::Format rpl_fmt_{[] {
    Moteus::Query::Format fmt;
    fmt.abs_position = mjbots::moteus::Resolution::kFloat;
    fmt.motor_temperature = mjbots::moteus::Resolution::kInt16;
    fmt.trajectory_complete = mjbots::moteus::Resolution::kInt8;
    return fmt;
  }()};

  Metro exec_metro_{10};
} basilisk;

class NeoKeyCommandReceiver {
 public:
  NeoKeyCommandReceiver(uint16_t interval_ms) : neokey_{0x30, &Wire} {
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
      Serial.println(" pressed. Basilisk CommandMode = SetPosition");
      basilisk.cmd_.mode = Basilisk::Command::Mode::SetPosition;
      basilisk.cmd_.set_position.position_l = 0;
      basilisk.cmd_.set_position.position_r = 0;
      basilisk.cmd_.set_position.progress =
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
} cmd_rcvr_neokey{5};

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
} rpl_sndr_serial_print;

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
  const uint32_t errorCode = can.begin(settings, [] { can.isr(); });
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
  cmd_rcvr_neokey.rcv();
  rpl_sndr_serial_print.snd();
  basilisk.execute();
}
