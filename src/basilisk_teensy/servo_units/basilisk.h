#pragma once

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
  Lps lps_;
  Imu imu_;
  Magnets mags_;
  LegoBlocks lego_;

  /////////////////////
  // Configurations: //

  const struct Configuration {
    struct {
      const int id_l = 1, id_r = 2;
      const uint8_t bus = 1;
    } servo;
    struct {
      const double c = 300.0, x_c = 300.0, y_c = 300.0;
    } lps;
    struct {
      const uint8_t pin_la = 3, pin_lt = 4, pin_ra = 5, pin_rt = 6;
      const uint32_t run_interval = 100;
    } magnets;
    struct {
      const int pin_l = 23, pin_r = 29;
      const uint32_t run_interval = 20;
    } lego;
  } cfg_;

  const PmCmd* const pm_cmd_template_;

  //////////////////
  // Constructor: //

  Basilisk(const Configuration& cfg)
      : cfg_{cfg},
        l_{cfg_.servo.id_l, cfg_.servo.bus, &globals::pm_fmt, &globals::q_fmt},
        r_{cfg_.servo.id_r, cfg_.servo.bus, &globals::pm_fmt, &globals::q_fmt},
        pm_cmd_template_{&globals::pm_cmd_template},
        lps_{cfg_.lps.c, cfg_.lps.x_c, cfg_.lps.y_c},
        imu_{},
        mags_{cfg_.magnets.pin_la, cfg_.magnets.pin_lt,  //
              cfg_.magnets.pin_ra, cfg_.magnets.pin_rt},
        lego_{cfg_.lego.pin_l, cfg_.lego.pin_r} {}

  ///////////////////
  // Setup method: //

  // Should be called before use.
  bool Setup() {
    if (!CanFdDriverInitializer::Setup(cfg_.servo.bus)) {
      Serial.println("Basilisk: CanFdDriver setup failed");
      return false;
    }
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
    return true;
  }

  ////////////////////////
  // Components runner: //

  void Run() {
    lps_.Run();
    imu_.Run();

    static utils::Beat mags_run_beat{cfg_.magnets.run_interval};
    if (mags_run_beat.Hit()) mags_.Run();

    static utils::Beat lego_run_beat{cfg_.lego.run_interval};
    if (lego_run_beat.Hit()) lego_.Run();
  }

  //////////////////////////////
  // Basilisk Command struct: //

  enum class MuxCR : bool { Xbee, Neokey } mux_cr = MuxCR::Neokey;

  struct Command {
    enum class Mode : uint8_t {
      Nop_Init,        // -> Nop_Idle
      Nop_Idle,        // .
      Wait,            // -> ExitMode
      Free,            // -> Wait(5s) -> Nop_Init
      SetRho,          // -> ExitMode
      Walk_InitLeft,   // -> SetRho -> Walk_InitRight
      Walk_InitRight,  // -> SetRho -> Walk_Step
      Walk_Step,       // -> SetRho -> Walk_Step(++step) ~> Nop_Init
      Diamond_Init,    // -> SetRho -> Diamond_Step
      Diamond_Step,    // -> SetRho -> Diamond_Step(++step) ~> Nop_Init
      Gee_Init,        // -> SetRho -> Gee_Step
      Gee_Step,        // -> SetRho -> Gee_Step(++step) ~> Nop_Init
      WalkToPos,       // ~> Nop_Init
      WalkToDir
    } mode;

    struct Wait {
      bool (*exit_condition)(Basilisk&);
      Mode exit_to_mode;
    } wait;

    struct SetRho {
      friend struct ModeRunners;

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

  void Print() {
    CommandBoth([](Servo& s) {
      s.SetQuery();
      s.Print();
    });
  }
};
