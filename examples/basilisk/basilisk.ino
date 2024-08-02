#include <Metro.h>
#include <initializers.h>
#include <neokey.h>
#include <servo.h>
#include <specific/neokey1x4_i2c0.h>
#include <specific/neokey3x4_i2c0.h>

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
           &q_fmt_} {}

  template <typename ServoCommand>
  void CommandBoth(ServoCommand c) {
    c(l_);
    c(r_);
  }

  struct Command {
    enum class Mode : uint8_t {
      Stop,
      Free,
      DExact075,
      SetPhi,
      Walk,
      Diamond,
      Gee
    } mode;

    struct Stop {
      enum class FSMState : bool { Init, Complete } fsm_state;
    } stop;

    struct Free {
      enum class FSMState : bool { Init, Complete } fsm_state;
    } free;

    struct DExact075 {
      enum class FSMState : bool { Init, Complete } fsm_state;
    } d_exact_075;

    struct SetPhi {
     public:
      enum class FSMState : uint8_t { Init, WaitComplete, Complete } fsm_state;
      double phi_l, phi_r;
    } set_phi;

    class Walk {
     public:
      enum class Progress : uint8_t {
        init,
        moving_left,
        moving_right,
        complete
      } progress;
      double stride = 0.125;
      uint8_t steps;

     private:
      friend class Basilisk;
      uint8_t current_step;
    } walk;

    struct Diamond {
      bool init;
    } diamond;

    struct Gee {
      bool init;
    } gee;

  } cmd_;

  void Executer() {
    using M = Command::Mode;

    CommandBoth([](Servo& s) { s.Query(); });

    switch (cmd_.mode) {
      case M::Stop: {
        ExecuteStop();
      } break;
      case M::Free: {
        ExecuteFree();
      } break;
      case M::DExact075: {
        ExecuteDExact075();
      } break;
      case M::SetPhi: {
        ExecuteSetPhi();
      } break;
      case M::Walk: {
        ExecuteWalk();
      } break;
      case M::Diamond: {
        ExecuteDiamond();
      }; break;
      case M::Gee: {
        ExecuteGee();
      } break;
    }
  }

  void ExecuteStop() {
    auto& c = cmd_.stop;
    using FSM = Command::Stop::FSMState;
    switch (c.fsm_state) {
      case FSM::Init: {
        Serial.println(F("ExecuteStop processing FSM::Init state"));
        CommandBoth([](Servo& s) { s.Stop(); });
        c.fsm_state = FSM::Complete;
      } break;
      default:
        break;
    }
  }

  void ExecuteFree() {
    auto& c = cmd_.free;
    using FSM = Command::Free::FSMState;
    switch (c.fsm_state) {
      case FSM::Init: {
        Serial.println(F("ExecuteFree processing FSM::Init state"));
        FreeL();
        FreeR();
        c.fsm_state = FSM::Complete;
      } break;
      default:
        break;
    }
  }

  void ExecuteDExact075() {  // 100ms blocking delay.
    auto& c = cmd_.d_exact_075;
    using FSM = Command::DExact075::FSMState;
    switch (c.fsm_state) {
      case FSM::Init: {
        Serial.println(F("ExecuteDExact075 processing FSM::Init state"));
        CommandBoth([](Servo& s) {
          s.Stop();
          delay(50);
          s.d(F("d exact 0.75"));
          delay(50);
          s.Stop();
        });
        c.fsm_state = FSM::Complete;
      } break;
      default:
        break;
    }
  }

  void ExecuteSetPhi() {
    auto& c = cmd_.set_phi;
    using FSM = Command::SetPhi::FSMState;
    switch (c.fsm_state) {
      case FSM::Init: {
        Serial.println(F("ExecuteSetPhi processing FSM::Init state"));
        l_.Position(c.phi_l);
        r_.Position(c.phi_r);
        c.fsm_state = FSM::WaitComplete;
      } break;
      case FSM::WaitComplete: {
        // Query is done at Executer before entering FSM-based stage.
        if (l_.trjcpt_ >> 2 | r_.trjcpt_ >> 2) {
          CommandBoth([](Servo& s) { s.Stop(); });
          c.fsm_state = FSM::Complete;
        }
        delay(500);
      } break;
      default:
        break;
    }
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
        CommandBoth([](Servo& s) { s.Stop(); });

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
        CommandBoth([&](Servo& s) { s.Position(0.75 - c.stride); });
        Print();
        /*
          // This does not work for unknown reason even though
          // the trjcpt_ values are correctly updating.
          while (l_.trjcpt_ < 4 || r_.trjcpt_ < 4) {
            CommandBoth([](Servo& s) {
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
        CommandBoth([](Servo& s) { s.Stop(); });

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
        CommandBoth([&](Servo& s) { s.Position(0.75 + c.stride); });
        /*
          // This does not work for unknown reason even though
          // the trjcpt_ values are correctly updating.
          while (l_.trjcpt_ < 4 || r_.trjcpt_ < 4) {
            CommandBoth([](Servo& s) {
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

  void ExecuteDiamond() {
    auto& c = cmd_.diamond;
    if (c.init) {
      // Move to initial position
      FixL();
      FreeR();
      l_.PositionWaitComplete(0.75);
      r_.PositionWaitComplete(0.75);
      Print();

      while (1) {
        // Step 1.
        FixL();
        FreeR();
        CommandBoth([&](Servo& s) { s.Position(1.125); });
        // Wait both complete.
        for (int temp_l = 0, temp_r = 0; temp_l < 4 || temp_r < 4;) {
          l_.Query();
          if (l_.GetReply().trajectory_complete) temp_l++;
          r_.Query();
          if (r_.GetReply().trajectory_complete) temp_r++;
          delay(10);
        }
        Print();
        delay(500);

        // Step 2.
        FixR();
        FreeL();
        CommandBoth([&](Servo& s) { s.Position(0.375); });
        // Wait both complete.
        for (int temp_l = 0, temp_r = 0; temp_l < 4 || temp_r < 4;) {
          l_.Query();
          if (l_.GetReply().trajectory_complete) temp_l++;
          r_.Query();
          if (r_.GetReply().trajectory_complete) temp_r++;
          delay(10);
        }
        Print();
        delay(500);

        // Step 3.
        FixL();
        FreeR();
        CommandBoth([&](Servo& s) { s.Position(0.625); });
        // Wait both complete.
        for (int temp_l = 0, temp_r = 0; temp_l < 4 || temp_r < 4;) {
          l_.Query();
          if (l_.GetReply().trajectory_complete) temp_l++;
          r_.Query();
          if (r_.GetReply().trajectory_complete) temp_r++;
          delay(10);
        }
        Print();
        delay(500);

        // Step 4.
        FixR();
        FreeL();
        CommandBoth([&](Servo& s) { s.Position(0.875); });
        // Wait both complete.
        for (int temp_l = 0, temp_r = 0; temp_l < 4 || temp_r < 4;) {
          l_.Query();
          if (l_.GetReply().trajectory_complete) temp_l++;
          r_.Query();
          if (r_.GetReply().trajectory_complete) temp_r++;
          delay(10);
        }
        Print();
        delay(500);
      }
    }
  }

  void ExecuteGee() {
    auto& c = cmd_.gee;
    if (c.init) {
      // Move to initial position
      FixL();
      FreeR();
      l_.PositionWaitComplete(0.75);
      r_.PositionWaitComplete(0.75);
      Print();

      while (1) {
        for (uint8_t i = 0; i < 3; i++) {
          FixLAnkle();
          FixRAnkle();
          FreeLToe();
          FreeRToe();
          CommandBoth([&](Servo& s) { s.Position(0.875); });
          // Wait both complete.
          for (int temp_l = 0, temp_r = 0; temp_l < 4 || temp_r < 4;) {
            l_.Query();
            if (l_.GetReply().trajectory_complete) temp_l++;
            r_.Query();
            if (r_.GetReply().trajectory_complete) temp_r++;
            delay(10);
          }
          Print();
          delay(500);

          FixLToe();
          FixRToe();
          FreeLAnkle();
          FreeRAnkle();
          CommandBoth([&](Servo& s) { s.Position(0.625); });
          // Wait both complete.
          for (int temp_l = 0, temp_r = 0; temp_l < 4 || temp_r < 4;) {
            l_.Query();
            if (l_.GetReply().trajectory_complete) temp_l++;
            r_.Query();
            if (r_.GetReply().trajectory_complete) temp_r++;
            delay(10);
          }
          Print();
          delay(500);
        }
        for (uint8_t i = 0; i < 3; i++) {
          FixLAnkle();
          FixRAnkle();
          FreeLToe();
          FreeRToe();
          CommandBoth([&](Servo& s) { s.Position(0.675); });
          // Wait both complete.
          for (int temp_l = 0, temp_r = 0; temp_l < 4 || temp_r < 4;) {
            l_.Query();
            if (l_.GetReply().trajectory_complete) temp_l++;
            r_.Query();
            if (r_.GetReply().trajectory_complete) temp_r++;
            delay(10);
          }
          Print();
          delay(500);

          FixLToe();
          FixRToe();
          FreeLAnkle();
          FreeRAnkle();
          CommandBoth([&](Servo& s) { s.Position(0.875); });
          // Wait both complete.
          for (int temp_l = 0, temp_r = 0; temp_l < 4 || temp_r < 4;) {
            l_.Query();
            if (l_.GetReply().trajectory_complete) temp_l++;
            r_.Query();
            if (r_.GetReply().trajectory_complete) temp_r++;
            delay(10);
          }
          Print();
          delay(500);
        }
      }
    }
  }

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

  void FixLAnkle() { digitalWrite(electromagnet_pins[0], LOW); }

  void FreeLAnkle() { digitalWrite(electromagnet_pins[0], HIGH); }

  void FixLToe() { digitalWrite(electromagnet_pins[1], LOW); }

  void FreeLToe() { digitalWrite(electromagnet_pins[1], HIGH); }

  void FixRAnkle() { digitalWrite(electromagnet_pins[2], LOW); }

  void FreeRAnkle() { digitalWrite(electromagnet_pins[2], HIGH); }

  void FixRToe() { digitalWrite(electromagnet_pins[3], LOW); }

  void FreeRToe() { digitalWrite(electromagnet_pins[3], HIGH); }

  void Print() {
    CommandBoth([](Servo& s) {
      s.Query();
      s.Print();
    });
  }

  Servo l_, r_;

  const uint8_t electromagnet_pins[4] = {3, 4, 5, 6};

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

    switch (key) {
      case 0: {  // Stop.
        mode = M::Stop;
        cmd.stop.fsm_state = C::Stop::FSMState::Init;
      } break;
      case 1: {  // Free.
        mode = M::Free;
        cmd.free.fsm_state = C::Free::FSMState::Init;
      } break;
      case 2: {  // "d exact 0.75".
        mode = M::DExact075;
        cmd.d_exact_075.fsm_state = C::DExact075::FSMState::Init;
      } break;
      case 3: {  // Set phi_l and phi_r.
        mode = M::SetPhi;
        cmd.set_phi.fsm_state = C::SetPhi::FSMState::Init;
        cmd.set_phi.phi_l = 0.0;
        cmd.set_phi.phi_r = 0.0;
      } break;
      case 4: {  // Walk in 90 degree stride.
        mode = M::Walk;
        cmd.walk.progress = C::Walk::Progress::init;
        cmd.walk.steps = 4;
        cmd.walk.stride = 0.125;
      } break;
      default:
        break;
    }

    // if (key == 0) {
    // } else if (key == 1) {
    //   // Set current positions as initial position.
    // } else if (key == 2) {
    //   // Gee.
    //   mode = M::Gee;
    //   cmd.gee.init = true;
    // } else if (key == 3) {
    //   // Diamond.
    //   mode = M::Diamond;
    //   cmd.diamond.init = true;
    // } else if (key == 4) {
    // } else if (key == 5) {
    //   // Catwalk.
    //   mode = M::Walk;
    //   cmd.walk.progress = C::Walk::Progress::init;
    //   cmd.walk.steps = 3;
    //   cmd.walk.stride = 0.24;
    // }
  }

  return 0;
}

auto& neokey = specific::neokey1x4_i2c0;
void NeokeyCommandReceiver() { neokey.read(); }

void SerialPrintReplySender() {
  basilisk.CommandBoth([](Servo& s) { s.Print(); });
}

void setup() {
  SerialInitializer.init();
  SpiInitializer.init();
  CanFdInitializer.init(CANFD_BUS);
  I2C0Initializer.init();
  I2C1Initializer.init();
  NeokeyInitializer.init(neokey);
  neokey.registerCallbackAll(neokey_cb);
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(basilisk.electromagnet_pins[i], OUTPUT);
  }

  basilisk.CommandBoth([](Servo& s) { s.Stop(); });
}

Metro executer_metro{10};
Metro receiver_metro{25};
Metro sender_metro{500};

void loop() {
  if (executer_metro.check()) basilisk.Executer();
  if (receiver_metro.check()) NeokeyCommandReceiver();
  if (sender_metro.check()) SerialPrintReplySender();
}
