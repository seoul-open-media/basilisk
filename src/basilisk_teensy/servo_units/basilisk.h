#pragma once

#include "../components/canfd_drivers.h"
#include "../components/imu.h"
#include "../components/lego_blocks.h"
#include "../components/lps.h"
#include "../components/magnets.h"
#include "../components/servo.h"
#include "../globals.h"
#include "../helpers/utils.h"

struct ModeRunners;

class Basilisk {
 public:
  /////////////////
  // Components: //

  Servo l_, r_;
  Lps lps_;          // Run continuously.
  Imu imu_;          // Run continuously.
  Magnets mags_;     // Run in regular interval.
  LegoBlocks lego_;  // Run in regular interval.
  utils::Beat mags_beat_;
  utils::Beat lego_beat_;

  /////////////////////
  // Configurations: //

  struct Configuration {
    struct {
      const double c, x_c, y_c;
    } lps;
    struct {
      const uint8_t pin_la, pin_lt, pin_ra, pin_rt;
      const uint32_t run_interval;
    } magnets;
    struct {
      const int pin_l, pin_r;
      const uint32_t run_interval;
    } lego;
  };

  const PmCmd* const pm_cmd_template_;

  //////////////////
  // Constructor: //

  Basilisk(const Configuration& cfg)
      : l_{1, 1, &globals::pm_fmt, &globals::q_fmt},
        r_{2, 1, &globals::pm_fmt, &globals::q_fmt},
        pm_cmd_template_{&globals::pm_cmd_template},
        lps_{cfg.lps.c, cfg.lps.x_c, cfg.lps.y_c},
        imu_{},
        mags_{cfg.magnets.pin_la, cfg.magnets.pin_lt,  //
              cfg.magnets.pin_ra, cfg.magnets.pin_rt},
        mags_beat_{cfg.magnets.run_interval},
        lego_{cfg.lego.pin_l, cfg.lego.pin_r},
        lego_beat_{cfg.lego.run_interval} {}

  ///////////////////
  // Setup method: //

  // Should be called before use.
  bool Setup() {
    if (!lps_.Setup()) {
      Serial.println("Basilisk: LPS setup failed");
      return false;
    }
    if (!imu_.Setup()) {
      Serial.println("Basilisk: IMU setup failed");
      return false;
    }
    if (!mags_.Setup()) {
      Serial.println("Basilisk: Magnets setup failed");
      return false;
    }
    if (!lego_.Setup()) {
      Serial.println("Basilisk: LegoBlocks setup failed");
      return false;
    }
    if (!CanFdDriverInitializer::Setup(1)) {
      Serial.println("Basilisk: CanFdDriver setup failed");
      return false;
    }
    CommandBoth([](Servo& s) { s.SetStop(); });
    Serial.println("Basilisk: Both Servos Stopped");

    Serial.println("Basilisk: All components setup succeeded");
    return true;
  }

  ////////////////////////
  // Components runner: //

  void Run() {
    lps_.Run();
    imu_.Run();

    if (mags_beat_.Hit()) mags_.Run();

    static utils::Beat lego_run_beat{20};
    if (lego_beat_.Hit()) lego_.Run();
  }

  //////////////////////////////
  // Basilisk Command struct: //

  enum class MuxCR : bool { Xbee, Neokey } mux_cr_ = MuxCR::Neokey;

  struct Command {
    enum class Mode {
      Idle_Init,       // -> Idle_Nop
      Idle_Nop,        // .
      Wait,            // -> ExitMode
      Free,            // -> Wait(3s) -> Idle_Init
      SetRho,          // -> ExitMode
      Walk_Init,       // -> Walk_InitLeft
      Walk_InitLeft,   // -> SetRho -> Walk_InitRight
      Walk_InitRight,  // -> SetRho -> Walk_Step
      Walk_Step,       // -> SetRho -> Walk_Step(++step) ~> Idle_Init
      Diamond_Init,    // -> SetRho -> Diamond_Step
      Diamond_Step,    // -> SetRho -> Diamond_Step(++step) ~> Idle_Init
      Gee_Init,        // -> SetRho -> Gee_Step
      Gee_Step,        // -> SetRho -> Gee_Step(++step) ~> Idle_Init
      WalkToPos,       // ~> Idle_Init
      WalkToDir        // .
    } mode = Mode::Idle_Init;

    struct Wait {
      bool (*exit_condition)(Basilisk&);
      Mode exit_to_mode;
    } wait;

    struct SetRho {
      friend struct ModeRunners;

      bool (*exit_condition)(Basilisk&);
      Mode exit_to_mode;

      void SetRhos(const double& _tgt_rho_l, const double& _tgt_rho_r) {
        tgt_rho_l =
            isnan(_tgt_rho_l) ? NaN : constrain(_tgt_rho_l, -0.625, 0.125);
        tgt_rho_r =
            isnan(_tgt_rho_r) ? NaN : constrain(_tgt_rho_r, -0.625, 0.125);
      }
      void SetVel(const double& _vel) { vel = abs(_vel); }
      void SetDampThr(const double& _damp_thr) { damp_thr = abs(_damp_thr); }

     private:
      double tgt_rho_l = -0.25, tgt_rho_r = -0.25;
      double vel = 0.1;
      double damp_thr = 0.1;
      const double fix_thr = 0.01;
    } set_rho;

    struct Walk {
      friend struct ModeRunners;

      Walk() {}
      Walk(double _stride, double _eightwalk_l, double _eightwalk_r,
           uint8_t _steps, bool _phase)
          : stride{_stride},
            eightwalk_l{_eightwalk_l},
            eightwalk_r{_eightwalk_r},
            steps{_steps},
            phase{_phase} {}

      // Delta theta from walk-standby pose to right-foot-forward pose.
      // Negative value manifests as walking backwards.
      double stride = 0.125;

      // Delta sigma from zero pose to walk-standby pose.
      double eightwalk_l = 0.0;
      double eightwalk_r = 0.0;

      // Total steps counting both left and right footsteps.
      uint8_t steps = 4;

      // false = moving right foot, true = moving left foot.
      bool phase = false;

     private:
      uint8_t current_step = 0;
    } walk;

    struct Diamond {
      friend struct ModeRunners;

      Diamond() {}
      Diamond(const double& _stride) : stride{_stride} {}

      // Half of top-bottom angle of the diamond.
      double stride = 0.125;

     private:
      uint8_t current_step = 0;

      // rho_l and rho_r for Step 0, 1, 2, 3.
      double tgt_rho(uint8_t step) {
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

      // Delta sigma between zero pose and shear pose.
      // Negative value manifests as moving left, and positive right.
      double stride = 0.125;

      // Total steps counting both ankle shears and toe shears.
      uint8_t steps = 8;

      // false = fix ankle, true = fix toe.
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

  void SetPositions(const double& rho_l, const double& rho_r) {
    l_.SetPosition([&] {
      auto pm_cmd = *pm_cmd_template_;
      pm_cmd.position = rho_l;
      return pm_cmd;
    }());
    r_.SetPosition([&] {
      auto pm_cmd = *pm_cmd_template_;
      pm_cmd.position = rho_r;
      return pm_cmd;
    }());
  }

  void Print() {
    CommandBoth([](Servo& s) {
      s.SetQuery();
      s.Print();
    });
  }
};
