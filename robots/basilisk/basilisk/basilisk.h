#pragma once

#include <Arduino.h>
#include <servo.h>

#include "components/ems.h"
#include "exec_funcs/exec_funcs_class.h"

#define CANFD_BUS 1

class Basilisk {
 public:
  Basilisk(int id_l = 1, int id_r = 2)
      : l_{id_l,
           CANFD_BUS,
           &pm_fmt_,
           &pm_cmd_template_,
           PmCmdPosRelTo::Absolute,
           &q_fmt_},
        r_{id_r,
           CANFD_BUS,
           &pm_fmt_,
           &pm_cmd_template_,
           PmCmdPosRelTo::Absolute,
           &q_fmt_} {}

  // Util methods:

  template <typename ServoCommand>
  void CommandBoth(ServoCommand c) {
    c(l_);
    c(r_);
  }

  bool BothComplete(const uint8_t& threshold) {
    return l_.trjcpt_ >= threshold && r_.trjcpt_ >= threshold;
  }

  void Print() {
    CommandBoth([](Servo& s) {
      s.Query();
      s.Print();
    });
  }

  // Basilisk Command struct:

  struct Command {
    enum class Mode : uint8_t {
      None,
      WaitTime,
      Stop,
      Em,
      DExactM025,  // Not preserved after power off.
      SetRho,
      Walk,
      Diamond,
      Gee
    } mode;

    struct WaitTime {
      friend class ExecFuncs;

      enum class FSMState : uint8_t { Wait } fsm_state = FSMState::Wait;

      uint32_t duration;
      Mode exit_to_mode;
      uint8_t exit_to_fsm;

     private:
      uint32_t init_time;
    } wait_time;

    struct Stop {
      enum class FSMState : uint8_t { Init } fsm_state = FSMState::Init;
    } stop;

    struct Em {
      enum class FSMState : uint8_t { Init } fsm_state = FSMState::Init;
      EmStrength strength[4] = {EmStrength::Max, EmStrength::Max,
                                EmStrength::Max, EmStrength::Max};
    } em;

    struct DExactM025 {
      enum class FSMState : uint8_t { Init } fsm_state = FSMState::Init;
    } d_exact_m025;

    struct SetRho {
      enum class FSMState : uint8_t { Init, Wait } fsm_state = FSMState::Init;
      double rho_l = -0.25, rho_r = -0.25;
    } set_rho;

    class Walk {
     public:
      Walk() {}

      Walk(double _stride, double _eightwalk_l, double _eightwalk_r,
           uint8_t _steps, bool _phase)
          : stride{_stride},
            eightwalk_l{_eightwalk_l},
            eightwalk_r{_eightwalk_r},
            steps{_steps},
            phase{_phase} {}

      enum class FSMState : uint8_t {
        Init,
        WaitInitLeft,
        WaitInitRight,
        Move,
        WaitMove
      } fsm_state = FSMState::Init;

      // Delta theta from walk-standby pose to right-foot-forward pose.
      // Negative value manifests as walking backwards.
      double stride = 0.125;

      // Delta sigma from zero pose to walk-standby pose.
      double eightwalk_l = 0.0;
      double eightwalk_r = 0.0;

      // Total steps counting both left and right footsteps.
      uint8_t steps = 4;

      // true = moving left foot; false = moving right foot.
      bool phase = false;

     private:
      friend class ExecFuncs;

      uint8_t current_step = 0;
    } walk;

    class Diamond {
     public:
      Diamond() {}

      Diamond(const double& _stride) : stride{_stride} {}

      enum class FSMState : uint8_t {
        Init,
        Step,
        Wait,
      } fsm_state = FSMState::Init;

      // Half of top-bottom angle of the diamond.
      double stride = 0.125;

     private:
      friend class ExecFuncs;

      uint8_t current_step = 0;

      // rho_l and rho_r for Step 0, 1, 2, 3.
      double target_rho(uint8_t step) {
        switch (step) {
          case 0: {
            return stride;
          } break;
          case 1: {
            return -0.5 - stride;
          } break;
          case 2: {
            return -0.5 + stride;
          } break;
          case 3: {
            return -stride;
          } break;
          default: {
            return stride;
          } break;
        }
      }
    } diamond;

    class Gee {
     public:
      Gee() {}

      Gee(const double& _stride, const uint8_t& _steps)
          : stride{_stride}, steps{_steps} {}

      enum class FSMState : uint8_t {
        Init,
        Step,
        Wait
      } fsm_state = FSMState::Init;

      // Delta sigma between zero pose and shear pose.
      double stride = 0.125;

      // Total steps counting both ankle shears and toe shears.
      uint8_t steps = 4;

      // false = fix ankle; true = fix toe.
      bool phase = false;

     private:
      friend class ExecFuncs;

      uint8_t current_step = 0;
    } gee;
  } cmd_;

  // Electric components:

  Servo l_, r_;
  Ems ems_;

 private:
  // PositionMode/Query Formats and templates:

  PmFmt pm_fmt_{.kp_scale = Res::kFloat,
                .maximum_torque = Res::kFloat,
                .watchdog_timeout = Res::kFloat,
                .velocity_limit = Res::kFloat,
                .accel_limit = Res::kFloat};

  PmCmd pm_cmd_template_{.kp_scale = 0.5,
                         .maximum_torque = 32.0,
                         .watchdog_timeout = NaN,
                         .velocity_limit = 0.5,
                         .accel_limit = 0.25};

  QFmt q_fmt_{[] {
    QFmt fmt;
    fmt.abs_position = Res::kFloat;
    fmt.motor_temperature = Res::kInt16;
    fmt.trajectory_complete = Res::kInt8;
    return fmt;
  }()};
} basilisk{2, 1};
