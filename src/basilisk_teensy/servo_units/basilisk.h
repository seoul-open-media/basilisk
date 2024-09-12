#pragma once

#include "../components/ems.h"
#include "../globals.h"
#include "../servo.h"

namespace basilisk {
struct ModeRunners;

class Basilisk {
 public:
  Basilisk(int id_l, int id_r, uint8_t bus = 1)
      : l_{id_l, bus, &globals::pm_fmt, &globals::q_fmt},
        r_{id_r, bus, &globals::pm_fmt, &globals::q_fmt},
        pm_cmd_template_{&globals::pm_cmd_template} {}

  /////////////////
  // Components: //

  Servo l_, r_;
  Magnets ems_;

  //////////////////////////////
  // Basilisk Command struct: //

  struct Command {
    enum class Mode : uint8_t {
      Nop,
      Wait,
      Stop,
      Em,
      SetRho,
      Walk,
      Diamond,
      Gee
    } mode;

    struct Wait {
      enum class FSMState : uint8_t { Init, Wait } fsm_state = FSMState::Init;
      bool (*exit_condition)(Basilisk&);
      Mode exit_to_mode;
      uint8_t exit_to_fsm;
    } wait;

    struct Stop {
      enum class FSMState : uint8_t { Init } fsm_state = FSMState::Init;
    } stop;

    struct Em {
      enum class FSMState : uint8_t { Init } fsm_state = FSMState::Init;
      MagnetStrength strength[4] = {MagnetStrength::Max, MagnetStrength::Max,
                                MagnetStrength::Max, MagnetStrength::Max};
    } em;

    struct SetRho {
      enum class FSMState : uint8_t { Init, Wait } fsm_state = FSMState::Init;
      double rho_l = -0.25, rho_r = -0.25;
    } set_rho;

    class Walk {
      friend struct ModeRunners;

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
      uint8_t current_step = 0;
    } walk;

    class Diamond {
      friend struct ModeRunners;

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
      uint8_t current_step = 0;

      // rho_l and rho_r for Step 0, 1, 2, 3.
      double target_rho(uint8_t step) {
        switch (step) {
          case 0:
            return stride;
          case 1:
            return -0.5 - stride;
          case 2:
            return -0.5 + stride;
          case 3:
            return -stride;
          default:
            return stride;
        }
      }
    } diamond;

    class Gee {
      friend struct ModeRunners;

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
      uint8_t steps = 8;

      // false = fix ankle; true = fix toe.
      bool phase = false;

     private:
      uint8_t current_step = 0;
    } gee;
  } cmd_;

  ///////////////////
  // Util methods: //

  template <typename ServoCommand>
  void CommandBoth(ServoCommand c) {
    c(l_);
    c(r_);
  }

  void Print() {
    CommandBoth([](Servo& s) {
      s.SetQuery();
      s.Print();
    });
  }

 private:
  /////////////////////
  // Configurations: //

  const PmCmd* const pm_cmd_template_;
};

}  // namespace basilisk
