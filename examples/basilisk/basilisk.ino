#include <Metro.h>
#include <initializers.h>
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

  bool BothComplete() { return l_.trjcpt_ >> 2 | r_.trjcpt_ >> 2; }

  struct Command {
    enum class Mode : uint8_t {
      Stop,
      ElectromagnetOnOff,
      DExactM025,
      SetRho,
      Walk,
      Diamond,
      Gee
    } mode;

    struct Stop {
      enum class FSMState : bool { Init, Complete } fsm_state;
    } stop;

    struct ElectromagnetOnOff {
      enum class FSMState : bool { Init, Complete } fsm_state;
    } electromagnet_on_off;

    struct DExactM025 {
      enum class FSMState : bool { Init, Complete } fsm_state;
    } d_exact_m025;

    struct SetRho {
      enum class FSMState : uint8_t { Init, Wait, Complete } fsm_state;
      double rho_l, rho_r;
    } set_rho;

    class Walk {
     public:
      enum class FSMState : uint8_t { Init, Move, Wait, Complete } fsm_state;
      double stride =
          0.125;  // Delta theta between zero pose and right-foot-forward pose.
      uint8_t steps;  // Total steps counting both left and right steps.

     private:
      friend class Basilisk;
      bool move_left;  // true = move left foot; false = move right foot;
      uint8_t current_step;
    } walk;

    class Diamond {
     public:
      enum class FSMState : uint8_t {
        Init,
        Step,
        Wait,
      } fsm_state;
      double stride = 0.125;  // Half of top-bottom angle of the diamond.

     private:
      friend class Basilisk;
      uint8_t current_step;
      double target_rhos[4];
    } diamond;

    class Gee {
     public:
      enum class FSMState : uint8_t { Init, Step, Wait, Complete } fsm_state;
      double stride = 0.125;  // Delta sigma between zero pose and shear pose.
      uint8_t steps;

     private:
      friend class Basilisk;
      uint8_t current_step;
      bool phase;  // false = fix ankle; true = fix toe.
    } gee;
  } cmd_;

  void Executer() {
    using M = Command::Mode;

    CommandBoth([](Servo& s) { s.Query(); });

    switch (cmd_.mode) {
      case M::Stop: {
        ExecuteStop();
      } break;
      case M::ElectromagnetOnOff: {
        ExecuteElectromagnetOnOff();
      } break;
      case M::DExactM025: {
        ExecuteDExactM025();
      } break;
      case M::SetRho: {
        ExecuteSetRho();
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
        Print();

        CommandBoth([](Servo& s) { s.Stop(); });
        c.fsm_state = FSM::Complete;
      } break;
      default:
        break;
    }
  }

  void ExecuteElectromagnetOnOff() {
    auto& c = cmd_.electromagnet_on_off;
    using FSM = Command::ElectromagnetOnOff::FSMState;
    switch (c.fsm_state) {
      case FSM::Init: {
        Serial.println(
            F("ExecuteElectromagnetOnOff processing FSM::Init state"));

        DigitalWriteElectromagnet(1, true);
        DigitalWriteElectromagnet(2, true);
        DigitalWriteElectromagnet(3, true);
        DigitalWriteElectromagnet(4, true);
        c.fsm_state = FSM::Complete;
      } break;
      default:
        break;
    }
  }

  void ExecuteDExactM025() {  // 200ms blocking delay, necessary for stability.
    auto& c = cmd_.d_exact_m025;
    using FSM = Command::DExactM025::FSMState;
    switch (c.fsm_state) {
      case FSM::Init: {
        Serial.println(F("ExecuteDExactM025 processing FSM::Init state"));

        CommandBoth([](Servo& s) {
          s.Stop();
          delay(50);
          s.d(F("d exact -0.25"));
          delay(50);
          s.Stop();
        });
        c.fsm_state = FSM::Complete;
      } break;
      default:
        break;
    }
  }

  void ExecuteSetRho() {
    auto& c = cmd_.set_rho;
    using FSM = Command::SetRho::FSMState;
    switch (c.fsm_state) {
      case FSM::Init: {
        Serial.println(F("ExecuteSetRho processing FSM::Init state"));

        l_.Position(c.rho_l);
        r_.Position(c.rho_r);
        c.fsm_state = FSM::Wait;
      } break;
      case FSM::Wait: {
        Serial.println(F("ExecuteSetRho processing FSM::Wait state"));

        // Query is done at Executer before entering FSM-based stage.
        if (l_.trjcpt_ >> 2 | r_.trjcpt_ >> 2) {
          CommandBoth([](Servo& s) { s.Stop(); });
          c.fsm_state = FSM::Complete;
        }
      } break;
      default:
        break;
    }
  }

  void ExecuteWalk() {
    auto& c = cmd_.walk;
    using FSM = Command::Walk::FSMState;
    switch (c.fsm_state) {
      case FSM::Init: {
        Serial.println(F("ExecuteWalk processing FSM::Init state"));
        Print();

        // Stop both Servos.
        Serial.println(F("Stop both Servos."));
        CommandBoth([](Servo& s) { s.Stop(); });
        Print();

        // Initialize current_step to 0.
        Serial.println(F("Initialize current_step to 0."));
        c.current_step = 0;
        Print();

        // Fix left foot and free right foot.
        Serial.println(F("Fix left foot and free right foot."));
        DigitalWriteElectromagnet(1, false);
        DigitalWriteElectromagnet(2, false);
        DigitalWriteElectromagnet(3, true);
        DigitalWriteElectromagnet(4, true);
        Print();

        // Fix rho_l.
        Serial.println(F("Fix rho_l."));
        l_.Position(NaN);
        Print();

        // Command rho_r to rho_l (walk standby pose)
        // and wait in FSM::Wait state. Exit to FSM::Move state when complete.
        Serial.println(F("Control rho_r to rho_l (walk standby pose)."));
        l_.Query();
        r_.Position(l_.GetReply().position);
        c.fsm_state = FSM::Wait;
        Print();
      } break;
      case FSM::Wait: {
        Serial.println(F("ExecuteWalk processing FSM::Wait state"));
        Print();

        // Resume to FSM::Move state
        // or FSM::Complete state if done walking all steps.
        if (BothComplete()) {
          if (c.current_step < c.steps) {
            c.fsm_state = FSM::Move;
          } else {
            c.fsm_state = FSM::Complete;
          }
        }
        Print();
      } break;
      case FSM::Move: {
        Serial.println(F("ExecuteWalk processing FSM::MoveLeft state"));
        Print();

        if (c.move_left) {
          // Fix right foot and free left foot.
          Serial.println(F("Fix right foot and free left foot."));
          DigitalWriteElectromagnet(1, true);
          DigitalWriteElectromagnet(2, true);
          DigitalWriteElectromagnet(3, false);
          DigitalWriteElectromagnet(4, false);
          Print();

          // Control rho_l and rho_r to -0.25 - stride.
          Serial.println(F("Control rho_l and rho_r to -0.25 - stride."));
          Print();
          CommandBoth([&](Servo& s) { s.Position(-0.25 - c.stride); });
          Print();

        } else {
          // Fix left foot and free right foot.
          Serial.println(F("Fix left foot and free right foot."));
          DigitalWriteElectromagnet(1, false);
          DigitalWriteElectromagnet(2, false);
          DigitalWriteElectromagnet(3, true);
          DigitalWriteElectromagnet(4, true);
          Print();

          // Control rho_l and rho_r to -0.25 + stride.
          Serial.println(F("Control rho_l and rho_r to -0.25 + stride."));
          Print();
          CommandBoth([&](Servo& s) { s.Position(-0.25 + c.stride); });
          Print();
        }

        // Update phase and current_step and jump to Wait state.
        Serial.println(
            F("Update phase and current_step and jump to Wait state."));
        c.current_step++;
        c.move_left = !c.move_left;
        c.fsm_state = FSM::Wait;
        Print();
      } break;
      case FSM::Complete: {
        Serial.println(F("ExecuteWalk processing FSM::Complete state"));
        Print();

        // Reverse. (temp)
        c.stride *= -1.0;
        c.fsm_state = FSM::Init;
      } break;
    }
  }

  void ExecuteDiamond() {
    auto& c = cmd_.diamond;
    using FSM = Command::Diamond::FSMState;
    switch (c.fsm_state) {
      case FSM::Init: {
        Serial.println(F("ExecuteDiamond processing FSM::Init state"));
        Print();

        // Fix left foot and free right foot.
        DigitalWriteElectromagnet(1, false);
        DigitalWriteElectromagnet(2, false);
        DigitalWriteElectromagnet(3, true);
        DigitalWriteElectromagnet(4, true);
        Print();

        // Move to initial position
        CommandBoth([&](Servo& s) { s.Position(-0.25); });
        Print();

        // Reset current_step to 0.
        c.current_step = 0;
        Print();

        // Set target rhos.
        c.target_rhos[0] = c.stride;
        c.target_rhos[1] = -0.5 - c.stride;
        c.target_rhos[2] = -0.5 + c.stride;
        c.target_rhos[3] = -c.stride;

        // Enter FSM::Wait state.
        c.fsm_state = FSM::Wait;
        Print();
      } break;
      case FSM::Wait: {
        Serial.println(F("ExecuteDiamond processing FSM::Wait state"));
        Print();

        // Resume to FSM::Step state when complete.
        if (BothComplete()) {
          c.fsm_state = FSM::Step;
        }
        Print();
      } break;
      case FSM::Step: {
        Serial.println(F("ExecuteDiamond processing FSM::Step state"));
        Serial.print(F("Current step: "));
        Serial.println(c.current_step);
        Print();

        if (c.current_step % 2 == 0) {
          // Fix right foot and free left foot.
          DigitalWriteElectromagnet(1, false);
          DigitalWriteElectromagnet(2, false);
          DigitalWriteElectromagnet(3, true);
          DigitalWriteElectromagnet(4, true);
        } else {
          // Fix right foot and free left foot.
          DigitalWriteElectromagnet(1, true);
          DigitalWriteElectromagnet(2, true);
          DigitalWriteElectromagnet(3, false);
          DigitalWriteElectromagnet(4, false);
        }
        Print();

        // Command both rhos to target.
        CommandBoth(
            [&](Servo& s) { s.Position(c.target_rhos[c.current_step]); });
        Print();

        // Increment current_step and enter FSM::Wait state.
        c.current_step++;
        if (c.current_step == 4) {
          c.current_step = 0;
        }
        c.fsm_state = FSM::Wait;
        Print();
      } break;
      default:
        break;
    }
  }

  void ExecuteGee() {
    auto& c = cmd_.gee;
    using FSM = Command::Gee::FSMState;
    switch (c.fsm_state) {
      case FSM::Init: {
        Serial.println(F("ExecuteGee processing FSM::Init state"));
        Print();

        // Fix left foot and free right foot
        DigitalWriteElectromagnet(1, false);
        DigitalWriteElectromagnet(2, false);
        DigitalWriteElectromagnet(3, true);
        DigitalWriteElectromagnet(4, true);
        Print();

        // Go to zero pose.
        CommandBoth([](Servo& s) { s.Position(-0.25); });

        // Reset current_step and phase.
        c.current_step = 0;
        c.phase = false;

        // Enter FSM::Wait state.
        c.fsm_state = FSM::Wait;
      } break;
      case FSM::Wait: {
        Serial.println(F("ExecuteGee processing FSM::Wait state"));
        Print();

        // Resume step when complete
        // or enter FSM::Complete state when all steps are done.
        if (BothComplete()) {
          if (c.current_step < c.steps) {
            c.fsm_state = FSM::Step;
          } else {
            c.fsm_state = FSM::Complete;
          }
        }
      } break;
      case FSM::Step: {
        if (c.phase) {
          // Fix toes and free ankles.
          DigitalWriteElectromagnet(1, true);
          DigitalWriteElectromagnet(2, false);
          DigitalWriteElectromagnet(3, true);
          DigitalWriteElectromagnet(4, false);

          // Shear.
          CommandBoth([&](Servo& s) { s.Position(-0.25 - c.stride); });
        } else {
          // Fix ankles and free toes.
          DigitalWriteElectromagnet(1, false);
          DigitalWriteElectromagnet(2, true);
          DigitalWriteElectromagnet(3, false);
          DigitalWriteElectromagnet(4, true);

          // Shear.
          CommandBoth([&](Servo& s) { s.Position(-0.25 + c.stride); });
        }

        // Update phase and current_step and enter FSM::Wait state.
        c.phase = !c.phase;
        if (!c.phase) {
          c.current_step++;
        }
        c.fsm_state = FSM::Wait;
      } break;
      default:
        break;
    }
  }

  void DigitalWriteElectromagnet(int id, bool free) {
    // ID    | 1         | 2       | 3          | 4
    // Index | 0         | 1       | 2          | 3
    // Part  | LeftAnkle | LeftToe | RightAnkle | RightToe
    digitalWrite(electromagnet_pins[id - 1], free ? HIGH : LOW);
  }

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

    Serial.print(F("Neokey rise: "));
    Serial.println(key);

    using C = Basilisk::Command;
    using M = C::Mode;
    auto& c = basilisk.cmd_;
    auto& m = basilisk.cmd_.mode;

    switch (key) {
      case 0: {  // Stop.
        m = M::Stop;
        c.stop.fsm_state = C::Stop::FSMState::Init;
      } break;
      case 1: {  // ElectromagnetOnOff.
        m = M::ElectromagnetOnOff;
        c.electromagnet_on_off.fsm_state =
            C::ElectromagnetOnOff::FSMState::Init;
      } break;
      case 2: {  // "d exact -0.25".
        m = M::DExactM025;
        c.d_exact_m025.fsm_state = C::DExactM025::FSMState::Init;
      } break;
      case 3: {  // Set rho_l and rho_r.
        m = M::SetRho;
        c.set_rho.fsm_state = C::SetRho::FSMState::Init;
        c.set_rho.rho_l = 0.0;
        c.set_rho.rho_r = 0.0;
      } break;
      case 4: {  // Walk in 45 degree stride.
        m = M::Walk;
        c.walk.fsm_state = C::Walk::FSMState::Init;
        c.walk.steps = 4;
        c.walk.stride = 0.125;
      } break;
      case 5: {  // Catwalk: walk in 90 degree stride.
        m = M::Walk;
        c.walk.fsm_state = C::Walk::FSMState::Init;
        c.walk.steps = 4;
        c.walk.stride = 0.25;
      } break;
      case 6: {  // Baby Walk: walk in 10 degree stride.
        m = M::Walk;
        c.walk.fsm_state = C::Walk::FSMState::Init;
        c.walk.steps = 16;
        c.walk.stride = 10.0 / 360.0;
      } break;
      case 7: {  // 8-walk.
        m = M::Walk;
        c.walk.fsm_state = C::Walk::FSMState::Init;
        c.walk.steps = 4;
        c.walk.stride = 0.25;
      } break;
      case 8: {  // Diamond step.
        m = M::Diamond;
        c.diamond.fsm_state = C::Diamond::FSMState::Init;
        c.diamond.stride = 0.125;
      } break;
      case 9: {  // Gee.
        m = M::Gee;
        c.gee.fsm_state = C::Gee::FSMState::Init;
        c.gee.stride = 0.125;
        c.gee.steps = 4;
      } break;
      default:
        break;
    }
  }

  return 0;
}

auto& neokey = specific::neokey3x4_i2c0;
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
Metro neokey_cr_metro{25};
Metro serial_print_rs_metro{500};

void loop() {
  if (executer_metro.check()) basilisk.Executer();
  if (neokey_cr_metro.check()) NeokeyCommandReceiver();
  if (serial_print_rs_metro.check()) SerialPrintReplySender();
}
