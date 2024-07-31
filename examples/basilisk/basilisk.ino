#include <Metro.h>
#include <initializers.h>
#include <neokey.h>
#include <servo.h>
#include <specific/neokey3x4_i2c1.h>

#define CANFD_BUS 1

class Basilisk {
 public:
  Basilisk()
      : l_{1,
           CANFD_BUS,
           &pm_fmt_,
           &pm_cmd_template_,
           CommandPositionRelativeTo::Absolute,
           &q_fmt_},
        r_{2,
           CANFD_BUS,
           &pm_fmt_,
           &pm_cmd_template_,
           CommandPositionRelativeTo::Absolute,
           &q_fmt_},
        lr_{l_, r_} {}

  template <typename ServoCommand>
  void CommandLR(ServoCommand c) {
    for (Servo& s : lr_) {
      c(s);
    }
  }

  struct Command {
    enum class Mode : uint8_t { Stop, DZero, SetPosition, Walk } mode;

    struct Stop {
      bool init;
    } stop;

    struct DZero {
      bool init;
    } d_zero;

    struct SetPosition {
      enum class Progress : uint8_t { init, moving, complete } progress;
      double position = 0.0;
      bool which_foot;  // 0 = left, 1 = right
    } set_position;

    struct Walk {
      enum class Progress : uint8_t {} progress;
      double speed = 1.0;
    } walk;
  } cmd_;

  void Executer() {
    using M = Command::Mode;

    CommandLR([](Servo& s) { s.Query(); });

    switch (cmd_.mode) {
      case M::Stop: {
        ExecuteStop();
      } break;
      case M::DZero: {
        ExecuteDZero();
      } break;
      case M::SetPosition: {
        ExecuteSetPosition();
      } break;
    }
  }

  void ExecuteStop() {
    auto& c = cmd_.stop;
    if (c.init) {
      Serial.println(F("ExecuteStop processing init state"));
      CommandLR([](Servo& s) { s.Stop(); });
      c.init = false;
    }
  }

  void ExecuteDZero() {
    auto& c = cmd_.d_zero;
    if (c.init) {
      Serial.println(F("ExecuteDZero processing init state"));
      CommandLR([](Servo& s) {
        s.Stop();
        delay(50);
        s.d(F("d exact 0"));
        delay(50);
        s.Stop();
      });
      c.init = false;
    }
  }

  void ExecuteSetPosition() {
    auto& c = cmd_.set_position;
    using P = Command::SetPosition::Progress;
    switch (c.progress) {
      case P::init: {
        Serial.println(F("ExecuteSetPosition processing init state"));
        CommandLR([&](Servo& s) { s.Position(c.position); });
        c.progress = P::moving;
      } break;
      case P::moving: {
        Serial.println(F("ExecuteSetPosition processing moving state"));
        for (Servo& s : lr_) {
          if (s.trjcpt_ < 4) {
            return;
          }
        }
        CommandLR([](Servo& s) { s.Stop(); });
        cmd_.set_position.progress = P::complete;
      } break;
    }
  }

  void Walk() {}

  Servo l_, r_;
  Servo lr_[2];

 private:
  PmFmt pm_fmt_{.maximum_torque = Res::kFloat,
                .velocity_limit = Res::kFloat,
                .accel_limit = Res::kFloat};

  PmCmd pm_cmd_template_{
      .maximum_torque = 32.0, .velocity_limit = 32.0, .accel_limit = 16.0};

  QFmt q_fmt_{[] {
    QFmt fmt;
    fmt.abs_position = Res::kFloat;
    fmt.motor_temperature = Res::kInt16;
    fmt.trajectory_complete = Res::kInt8;
    return fmt;
  }()};
} basilisk;

NeoKey1x4Callback neokey_cb(keyEvent evt) {
  if (evt.bit.EDGE == SEESAW_KEYPAD_EDGE_RISING) {
    auto key = evt.bit.NUM;

    Serial.print(F("Rise: "));
    Serial.println(key);

    using C = Basilisk::Command;
    using M = C::Mode;
    auto& cmd = basilisk.cmd_;
    auto& mode = basilisk.cmd_.mode;

    if (key <= 0) {
      mode = M::Stop;
      cmd.stop.init = true;
    } else if (key <= 1) {
      mode = M::DZero;
      cmd.d_zero.init = true;
    } else {
      mode = M::SetPosition;
      cmd.set_position.progress = C::SetPosition::Progress::init;
      cmd.set_position.position = key * 0.25;
    }
  }

  return 0;
}

auto& neokey = specific::neokey3x4_i2c1;
void NeokeyCommandReceiver() { neokey.read(); }

void SerialPrintReplySender() {
  basilisk.CommandLR([](Servo& s) { s.Print(); });
}

void setup() {
  SerialInitializer.init();
  SpiInitializer.init();
  CanFdInitializer.init(CANFD_BUS);
  NeokeyInitializer.init(neokey);
  neokey.registerCallbackAll(neokey_cb);

  basilisk.CommandLR([](Servo& s) { s.Stop(); });
}

Metro executer_metro{10};
Metro receiver_metro{25};
Metro sender_metro{500};

void loop() {
  if (executer_metro.check()) basilisk.Executer();
  if (receiver_metro.check()) NeokeyCommandReceiver();
  if (sender_metro.check()) SerialPrintReplySender();
}
