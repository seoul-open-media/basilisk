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
  Servo* lr_[2] = {nullptr, nullptr};
  Lps lps_;          // Run continuously.
  Imu imu_;          // Run continuously.
  Magnets mags_;     // Run in regular interval.
  LegoBlocks lego_;  // Run in regular interval.
  utils::Beat mags_beat_;
  utils::Beat lego_beat_;

  ////////////////////
  // Configuration: //

  struct Config {
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
  const double gear_rat_ = 21.0;  // rotor = output * gear_rat_

  //////////////////
  // Constructor: //

  Basilisk(const Config& cfg)
      : l_{1, 1, &globals::pm_fmt, &globals::q_fmt},
        r_{2, 1, &globals::pm_fmt, &globals::q_fmt},
        lr_{&l_, &r_},
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
      SetPhi,          // -> ExitMode
      SetMags,         // -> Wait -> ExitMode
      Walk_Init,       // -> Walk_InitLeft
      Walk_InitLeft,   // -> SetPhi -> Walk_InitRight
      Walk_InitRight,  // -> SetPhi -> Walk_Step
      Walk_Step,       // -> SetPhi -> Walk_Step(++step) ~> Idle_Init
      Diamond_Init,    // -> SetPhi -> Diamond_Step
      Diamond_Step,    // -> SetPhi -> Diamond_Step(++step) ~> Idle_Init
      Gee_Init,        // -> SetPhi -> Gee_Step
      Gee_Step,        // -> SetPhi -> Gee_Step(++step) ~> Idle_Init
      WalkToPos,       // ~> Idle_Init
      WalkToDir        // .
    } mode = Mode::Idle_Init;

    struct Wait {
      uint32_t init_time;
      bool (*exit_condition)(Basilisk*);
      Mode exit_to_mode;
    } wait;

    struct SetPhi {
      friend struct ModeRunners;

      Mode exit_to_mode;

      void SetPhis(const double& _tgt_phi_l, const double& _tgt_phi_r) {
        tgt_phi[0] =
            isnan(_tgt_phi_l) ? NaN : constrain(_tgt_phi_l, -0.625, 0.125);
        tgt_phi[1] =
            isnan(_tgt_phi_r) ? NaN : constrain(_tgt_phi_r, -0.625, 0.125);
      }
      void SetPhiDots(const double& _phidot_l, const double& _phidot_r) {
        tgt_phidot[0] = constrain(abs(_phidot_l), 0.0, 0.75);
        tgt_phidot[1] = constrain(abs(_phidot_r), 0.0, 0.75);
      }
      void SetPhiDDots(const double& _phiddot_l, const double& _phiddot_r) {
        tgt_phiddot[0] = constrain(abs(_phiddot_l), 0.0, 3.0);
        tgt_phiddot[1] = constrain(abs(_phiddot_r), 0.0, 3.0);
      }
      void SetDampThr(const double& _damp_thr) { damp_thr = abs(_damp_thr); }

     private:
      double tgt_phi[2] = {-0.25, -0.25};  // [0]: l, [1]: r
      double tgt_phidot[2] = {0.1, 0.1};   // [0]: l, [1]: r
      double tgt_phiddot[2] = {0.1, 0.1};  // [0]: l, [1]: r
      double damp_thr = 0.1;
      const double fix_thr = 0.01;
    } set_phi;

    struct SetMags {
      Mode exit_mode;

      MagnetStrength strengths[4] = {MagnetStrength::Max, MagnetStrength::Max,
                                     MagnetStrength::Max, MagnetStrength::Max};
      bool expected_contact[2] = {true, true};
      uint8_t expected_consec_verif[2] = {32, 32};
      uint32_t min_wait_time = 100;
      uint32_t max_wait_time = 10000;
    } set_mags;

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

      // phi_l and phi_r for Step 0, 1, 2, 3.
      double tgt_phi(uint8_t step) {
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
    for (auto* s : lr_) c(*s);
  }

  void Print() {
    CommandBoth([](Servo& s) {
      s.SetQuery();
      s.Print();
    });
  }
};
