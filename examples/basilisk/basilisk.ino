#include <ACAN2517FD.h>
#include <Adafruit_NeoKey_1x4.h>
#include <Metro.h>
#include <Moteus.h>
#include <Wire.h>
#include <initializers.h>
#include <neokey.h>
#include <servo.h>

class Basilisk {
 public:
  Basilisk()
      : l_{1, &pm_fmt_, &q_fmt_}, r_{2, &pm_fmt_, &q_fmt_}, exec_metro_{10} {}

  void execute() {
    if (!exec_metro_.check()) return;

    CommandLR([](Moteus* m) { m->SetQuery(); });

    switch (cmd_.mode) {
      case Command::Mode::Stop: {
      
        break;
      }
      case Command::Mode::DZero: {
        break;
      }
      case Command::Mode::SetPositions: {
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
      DZero,
      SetPositions,
      WalkForward,
      WalkBackward,
      GoToLpsPosition,
      Sine
    } mode;

    struct {  // don't use union since progress data should be reserved
              // or previous command value should be used for comparing with new
              // value
      struct Stop {
        bool waiting;
      } stop;

      struct DZero {
        bool waiting;
      } d_zero;

      struct SetPositions {
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
    } cmd;
  } cmd_;

  Servo l_, r_;
  PmFmt pm_fmt_;
  PmCmd pm_cmd_template_;
  QFmt q_fmt_;

 private:
  Metro exec_metro_;
} basilisk;

class NeokeyReceiver {
 public:
  NeokeyReceiver(uint16_t interval_ms) : neokey_{0x30, &Wire} {
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
      Serial.println(" pressed. Basilisk CommandMode = DZero");
      basilisk.cmd_.mode = Basilisk::Command::Mode::DZero;
      basilisk.cmd_.d_zero.waiting = true;
    } else if (reading & (1 << 2)) {
      Serial.print("NeoKey #");
      Serial.print(2);
      Serial.println(" pressed. Basilisk CommandMode = SetPositions");
      basilisk.cmd_.mode = Basilisk::Command::Mode::SetPositions;
      basilisk.cmd_.set_positions.l = 0;
      basilisk.cmd_.set_positions.r = 0;
      basilisk.cmd_.set_positions.progress =
          Basilisk::Command::SetPositions::Progress::waiting;
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
} neokey_cr{5};

void setup() {
  SerialInitializer.init();
  I2C0Initializer.init();
  SpiInitializer.init();
  Neokey0Initializer.init();
  CanFdInitializer.init();

  basilisk.mot_l_.SetStop();
  basilisk.mot_r_.SetStop();
}

void loop() {
  while (1) {
    basilisk.mot_l_.SetPosition({.position = 0});
  }

  neokey_cr.rcv();
  serial_print_rpl_sndr.snd();
  basilisk.execute();
}
