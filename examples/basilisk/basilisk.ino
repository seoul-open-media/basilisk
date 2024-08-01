#include <Metro.h>
#include <initializers.h>
#include <neokey.h>
#include <servo.h>
#include <specific/neokey1x4_i2c0.h>

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
    enum class Mode : uint8_t {
      Stop,
      ExecuteDExact075,
      SetPosition,
      Walk
    } mode;

    struct Stop {
      bool init;
    } stop;

    struct ExecuteDExact075 {
      bool init;
    } d_zero;

    struct SetPosition {
      enum class Progress : uint8_t { init, moving, complete } progress;
      double position;
    } set_position;

    struct Walk {
      enum class Progress : uint8_t {
        init,
        moving_left,
        moving_right,
        complete
      } progress;
      double stride = 0.125;
      uint8_t steps;
      uint8_t current_step;
    } walk;
  } cmd_;

  void Executer() {
    using M = Command::Mode;

    CommandLR([](Servo& s) { s.Query(); });

    switch (cmd_.mode) {
      case M::Stop: {
        ExecuteStop();
      } break;
      case M::ExecuteDExact075: {
        ExecuteDExact075();
      } break;
      case M::SetPosition: {
        ExecuteSetPosition();
      } break;
      case M::Walk: {
        ExecuteWalk();
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

  void ExecuteDExact075() {
    auto& c = cmd_.d_zero;
    if (c.init) {
      Serial.println(F("ExecuteDExact075 processing init state"));
      CommandLR([](Servo& s) {
        s.Stop();
        delay(50);
        s.d(F("d exact 0.75"));
        delay(50);
        s.Stop();
      });
      c.init = false;
    }
  }

  void ExecuteSetPosition() {
    auto& c = cmd_.set_position;
    using P = Command::SetPosition::Progress;
    switch (c.progress) {}
  }

  void ExecuteWalk() {
    auto& c = cmd_.walk;
    using P = Command::Walk::Progress;
    switch (c.progress) {
      Serial.println(F("ExecuteWalk processing init state"));

      Serial.println(F("ExecuteWalk processing init state"));
      case P::init: {
        // Stop both Servos.
        Serial.println(F("Stop both Servos."));
        CommandLR([](Servo& s) { s.Stop(); });

        // Initialize current_step to 0.
        Serial.println(F("Initialize current_step to 0."));
        c.current_step = 0;
        Print();

        // Fix sig_L and free sig_R.
        Serial.println(F("Fix sig_L and free sig_R."));
        FixL();
        FreeR();
        Print();

        // Fix phi_L.
        Serial.println(F("Fix phi_L."));
        l_.Position(NaN);
        Print();

        // Control phi_R to phi_L.
        Serial.println(F("Control phi_R to phi_L."));
        l_.Query();
        r_.PositionWaitComplete(l_.GetReply().position);
        Print();

        // Jump to moving_right state.
        Serial.println(F("Jump to moving_right state."));
        c.progress = P::moving_right;
        Print();
      } break;
      case P::moving_left: {
        Serial.println(F("ExecuteWalk processing moving_left state"));
        Print();

        // Fix sig_R and free sig_L.
        Serial.println(F("Fix sig_R and free sig_L."));
        FixR();
        FreeL();
        Print();

        // Control phi_L and phi_R to 5/8.
        CommandLR([&](Servo& s) { s.Position(0.75 - c.stride); });
        Print();
        /*
          // This does not work for unknown reason even though
          // the trjcpt_ values are correctly updating.
          while (l_.trjcpt_ < 4 || r_.trjcpt_ < 4) {
            CommandLR([](Servo& s) {
              s.Query();
              // s.Print();
            });
            delay(10);
          }
        */
        // Wait both complete.
        for (int temp_l = 0, temp_r = 0; temp_l < 4 || temp_r < 4;) {
          l_.Query();
          if (l_.GetReply().trajectory_complete) temp_l++;
          r_.Query();
          if (r_.GetReply().trajectory_complete) temp_r++;
          delay(10);
        }
        Print();

        // Left foot forward complete.
        CommandLR([](Servo& s) { s.Stop(); });

        // Jump.
        c.current_step++;
        if (c.current_step < c.steps) {
          c.progress = P::moving_right;
        } else {
          c.progress = P::complete;
        }
        Print();
      } break;
      case P::moving_right: {
        Serial.println(F("ExecuteWalk processing moving_right state"));
        Print();

        // Fix sig_L and free sig_R.
        Serial.println(F("Fix sig_L and free sig_R."));
        FixL();
        FreeR();
        Print();

        // Control phi_L and phi_R to 7/8.
        Serial.println(F("Control phi_L and phi_R to 7/8."));
        Print();
        CommandLR([&](Servo& s) { s.Position(0.75 + c.stride); });
        /*
          // This does not work for unknown reason even though
          // the trjcpt_ values are correctly updating.
          while (l_.trjcpt_ < 4 || r_.trjcpt_ < 4) {
            CommandLR([](Servo& s) {
              s.Query();
              // s.Print();
            });
            delay(10);
          }
        */
        // Wait both complete.
        for (int temp_l = 0, temp_r = 0; temp_l < 4 || temp_r < 4;) {
          l_.Query();
          if (l_.GetReply().trajectory_complete) temp_l++;
          r_.Query();
          if (r_.GetReply().trajectory_complete) temp_r++;
          delay(10);
        }
        Print();

        // Jump.
        c.current_step++;
        if (c.current_step < c.steps) {
          c.progress = P::moving_left;
        } else {
          c.progress = P::complete;
        }
        Print();
      } break;
      case P::complete: {
        Serial.println(F("ExecuteWalk processing complete state"));

        // Reverse.
        c.stride *= -1.0;
        c.progress = P::init;
      } break;
    }
  }

  Servo l_, r_;
  Servo lr_[2];

  const uint8_t electromagnet_pins[4] = {3, 4, 5, 6};

  void FixL() {
    digitalWrite(electromagnet_pins[0], LOW);
    digitalWrite(electromagnet_pins[1], LOW);
  }

  void FreeL() {
    digitalWrite(electromagnet_pins[0], HIGH);
    digitalWrite(electromagnet_pins[1], HIGH);
  }

  void FixR() {
    digitalWrite(electromagnet_pins[2], LOW);
    digitalWrite(electromagnet_pins[3], LOW);
  }

  void FreeR() {
    digitalWrite(electromagnet_pins[2], HIGH);
    digitalWrite(electromagnet_pins[3], HIGH);
  }

  void Print() {
    CommandLR([](Servo& s) {
      s.Query();
      s.Print();
    });
  }

 private:
  PmFmt pm_fmt_{.maximum_torque = Res::kFloat,
                .watchdog_timeout = Res::kFloat,
                .velocity_limit = Res::kFloat,
                .accel_limit = Res::kFloat};

  PmCmd pm_cmd_template_{.maximum_torque = 32.0,
                         .watchdog_timeout = NaN,
                         .velocity_limit = 2.0,
                         .accel_limit = 1.0};

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

    if (key == 0) {
      mode = M::Stop;
      cmd.stop.init = true;
    } else if (key == 1) {
      mode = M::Stop;
      cmd.stop.init = true;
      basilisk.FreeL();
      basilisk.FreeR();
    } else if (key == 2) {
      mode = M::ExecuteDExact075;
      cmd.d_zero.init = true;
    } else {
      mode = M::Walk;
      cmd.walk.progress = C::Walk::Progress::init;
      cmd.walk.steps = 3;
      cmd.walk.stride = 0.24;
    }
  }

  return 0;
}

auto& neokey = specific::neokey1x4_i2c0;
void NeokeyCommandReceiver() { neokey.read(); }

void SerialPrintReplySender() {
  basilisk.CommandLR([](Servo& s) { s.Print(); });
}

void setup() {
  SerialInitializer.init();
  SpiInitializer.init();
  CanFdInitializer.init(CANFD_BUS);
  I2C0Initializer.init();
  NeokeyInitializer.init(neokey);
  neokey.registerCallbackAll(neokey_cb);

  for (uint8_t i = 0; i < 4; i++) {
    pinMode(basilisk.electromagnet_pins[i], OUTPUT);
  }

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
