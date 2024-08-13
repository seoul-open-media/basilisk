#include <beat.h>
#include <initializers.h>
#include <servo.h>
#include <specific/neokey3x4_i2c0.h>

#define CANFD_BUS 1

class Basilisk {
 public:
  Basilisk()
      : l_{1,
           CANFD_BUS,
           &pm_fmt_,
           &pm_cmd_template_,
           PmCmdPosRelTo::Absolute,
           &q_fmt_},
        r_{2,
           CANFD_BUS,
           &pm_fmt_,
           &pm_cmd_template_,
           PmCmdPosRelTo::Absolute,
           &q_fmt_} {}

  template <typename ServoCommand>
  void CommandBoth(ServoCommand c) {
    c(l_);
    c(r_);
  }

  bool BothComplete() { return l_.trjcpt_ >> 2 && r_.trjcpt_ >> 2; }

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
      enum class FSMState : uint8_t {
        Init,
        WaitInitLeft,
        WaitInitRight,
        Move,
        WaitMove,
        Complete
      } fsm_state;
      double eightwalk_l = 0.0,  // Delta sigma from zero pose
          eightwalk_r = 0.0;     // to walk-standby pose.
      double stride = 0.125;     // Delta theta from walk-standby pose
                                 // to right-foot-forward pose. Negative value
                                 // manifests as walking backwards.
      uint8_t steps;  // Total steps counting both left and right footsteps.

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
      default:
        break;
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

        EmFixAll();  // Temporary.

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

        EmFreeAll();  // Temporary.

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
        if (BothComplete()) {
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

        // Initialize private state values.
        Serial.println(F("Initialize private state values."));
        c.current_step = 0;
        c.move_left = false;
        Print();

        // Initialize left foot and enter FSM::WaitInitLeft state.
        Serial.println(F("Initialize left foot."));
        ElectromagnetDigitalWrite(1, true);
        ElectromagnetDigitalWrite(2, true);
        ElectromagnetDigitalWrite(3, false);
        ElectromagnetDigitalWrite(4, false);
        r_.Position(NaN);
        l_.Position(-0.25 - c.eightwalk_l);
        c.fsm_state = FSM::WaitInitLeft;
        Print();
      } break;
      case FSM::WaitInitLeft: {
        // When left foot initialization is complete,
        // initialize right foot and enter FSM::WaitInitRight state.
        Serial.println(F("ExecuteWalk processing FSM::WaitInitLeft state"));
        Print();

        if (BothComplete()) {
          Serial.println(F("Left foot initialization complete."));
          Serial.println(F("Initialize right foot."));
          ElectromagnetDigitalWrite(1, false);
          ElectromagnetDigitalWrite(2, false);
          ElectromagnetDigitalWrite(3, true);
          ElectromagnetDigitalWrite(4, true);
          Print();

          l_.Position(NaN);
          r_.Position(-0.25 - c.eightwalk_r);
          c.fsm_state = FSM::WaitInitRight;
          Print();
        }
      } break;
      case FSM::WaitInitRight: {
        if (BothComplete()) {
          Serial.println(F("Right foot initialization complete."));

          c.fsm_state = FSM::Move;
          Print();
        }
      } break;
      case FSM::Move: {
        Serial.println(F("ExecuteWalk processing FSM::Move state"));
        Print();

        if (c.move_left) {
          // Fix right foot and free left foot.
          Serial.println(F("Fix right foot and free left foot"));
          ElectromagnetDigitalWrite(1, true);
          ElectromagnetDigitalWrite(2, true);
          ElectromagnetDigitalWrite(3, false);
          ElectromagnetDigitalWrite(4, false);
          Print();

          // Control rhos.
          Serial.println(F("Control rhos"));
          l_.Position(-0.25 - c.eightwalk_l - c.stride);
          r_.Position(-0.25 - c.eightwalk_r - c.stride);
          Print();
        } else {
          // Fix left foot and free right foot.
          Serial.println(F("Fix left foot and free right foot."));
          ElectromagnetDigitalWrite(1, false);
          ElectromagnetDigitalWrite(2, false);
          ElectromagnetDigitalWrite(3, true);
          ElectromagnetDigitalWrite(4, true);
          Print();

          // Control rhos.
          Serial.println(F("Control rhos"));
          l_.Position(-0.25 - c.eightwalk_l + c.stride);
          r_.Position(-0.25 - c.eightwalk_r + c.stride);
          Print();
        }

        // Update phase and current_step and jump to WaitMove state.
        Serial.println(
            F("Update phase and current_step and jump to WaitMove state."));
        c.current_step++;
        c.move_left = !c.move_left;
        c.fsm_state = FSM::WaitMove;
        Print();
      } break;
      case FSM::WaitMove: {
        Serial.println(F("ExecuteWalk processing FSM::WaitMove state"));
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
      case FSM::Complete: {
        Serial.println(F("ExecuteWalk processing FSM::Complete state"));
        Print();

        // Temporarily reverse.
        c.stride *= -1.0;
        c.fsm_state = FSM::Init;
      } break;
      default:
        break;
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
        ElectromagnetDigitalWrite(1, false);
        ElectromagnetDigitalWrite(2, false);
        ElectromagnetDigitalWrite(3, true);
        ElectromagnetDigitalWrite(4, true);
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
          // Fix left foot and free right foot.
          ElectromagnetDigitalWrite(1, false);
          ElectromagnetDigitalWrite(2, false);
          ElectromagnetDigitalWrite(3, true);
          ElectromagnetDigitalWrite(4, true);
        } else {
          // Fix right foot and free left foot.
          ElectromagnetDigitalWrite(1, true);
          ElectromagnetDigitalWrite(2, true);
          ElectromagnetDigitalWrite(3, false);
          ElectromagnetDigitalWrite(4, false);
        }
        Print();

        // Command both rhos to target.
        Serial.print(F("Command both rhos to target: "));
        Serial.println(c.target_rhos[c.current_step]);
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
        ElectromagnetDigitalWrite(1, false);
        ElectromagnetDigitalWrite(2, false);
        ElectromagnetDigitalWrite(3, true);
        ElectromagnetDigitalWrite(4, true);
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
          ElectromagnetDigitalWrite(1, true);
          ElectromagnetDigitalWrite(2, false);
          ElectromagnetDigitalWrite(3, true);
          ElectromagnetDigitalWrite(4, false);

          // Shear.
          CommandBoth([&](Servo& s) { s.Position(-0.25 - c.stride); });
        } else {
          // Fix ankles and free toes.
          ElectromagnetDigitalWrite(1, false);
          ElectromagnetDigitalWrite(2, true);
          ElectromagnetDigitalWrite(3, false);
          ElectromagnetDigitalWrite(4, true);

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

  void ElectromagnetDigitalWrite(int id, bool free) {
    // ID    | 1         | 2       | 3          | 4
    // Index | 0         | 1       | 2          | 3
    // Part  | LeftAnkle | LeftToe | RightAnkle | RightToe
    digitalWrite(electromagnet_pins[id - 1], free);
  }

  void EmFixAll() {
    ElectromagnetDigitalWrite(1, false);
    ElectromagnetDigitalWrite(2, false);
    ElectromagnetDigitalWrite(3, false);
    ElectromagnetDigitalWrite(4, false);
  }

  void EmFreeAll() {
    ElectromagnetDigitalWrite(1, true);
    ElectromagnetDigitalWrite(2, true);
    ElectromagnetDigitalWrite(3, true);
    ElectromagnetDigitalWrite(4, true);
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
                         .velocity_limit = 4.0,
                         .accel_limit = 1.0};

  QFmt q_fmt_{[] {
    QFmt fmt;
    fmt.abs_position = Res::kFloat;
    fmt.motor_temperature = Res::kInt16;
    fmt.trajectory_complete = Res::kInt8;
    return fmt;
  }()};
} basilisk;

auto& neokey = specific::neokey3x4_i2c0;
void neokey_cb(uint16_t key) {
  Serial.print(F("Neokey rose: "));
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
      c.electromagnet_on_off.fsm_state = C::ElectromagnetOnOff::FSMState::Init;
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
      c.walk.eightwalk_l = 0.0;
      c.walk.eightwalk_r = 0.0;
    } break;
    case 5: {  // Catwalk: walk in 90 degree stride.
      m = M::Walk;
      c.walk.fsm_state = C::Walk::FSMState::Init;
      c.walk.steps = 4;
      c.walk.stride = 0.25;
      c.walk.eightwalk_l = 0.0;
      c.walk.eightwalk_r = 0.0;
    } break;
    case 6: {  // Baby Walk: walk in 10 degree stride.
      m = M::Walk;
      c.walk.fsm_state = C::Walk::FSMState::Init;
      c.walk.steps = 16;
      c.walk.stride = 10.0 / 360.0;
      c.walk.eightwalk_l = 0.0;
      c.walk.eightwalk_r = 0.0;
    } break;
    case 7: {  // 8-walk.
      m = M::Walk;
      c.walk.fsm_state = C::Walk::FSMState::Init;
      c.walk.steps = 4;
      c.walk.stride = 0.125;
      c.walk.eightwalk_l = 0.125;
      c.walk.eightwalk_r = -0.125;
    } break;
    case 8: {  // Diamond step.
      m = M::Diamond;
      c.diamond.fsm_state = C::Diamond::FSMState::Init;
      c.diamond.stride = 0.125;
    } break;
    case 9: {  // Gee in right direction.
      m = M::Gee;
      c.gee.fsm_state = C::Gee::FSMState::Init;
      c.gee.stride = 0.125;
      c.gee.steps = 4;
    } break;
    case 10: {  // Gee in left direction.
      m = M::Gee;
      c.gee.fsm_state = C::Gee::FSMState::Init;
      c.gee.stride = -0.125;
      c.gee.steps = 4;
    }
    default:
      break;
  }
}
void NeokeyCommandReceiver() { neokey.Read(); }

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
  neokey.SetCommonRiseCallback(neokey_cb);
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(basilisk.electromagnet_pins[i], OUTPUT);
  }

  basilisk.CommandBoth([](Servo& s) { s.Stop(); });
}

Beat executer_beat{10};
Beat neokey_cr_beat{25};
Beat serial_print_rs_beat{500};

void loop() {
  if (executer_beat.Hit()) basilisk.Executer();
  if (neokey_cr_beat.Hit()) NeokeyCommandReceiver();
  if (serial_print_rs_beat.Hit()) SerialPrintReplySender();
}
