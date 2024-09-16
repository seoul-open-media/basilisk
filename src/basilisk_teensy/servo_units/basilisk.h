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
  Lps lps_;          // Run every loop().
  Imu imu_;          // Run every loop().
  LegoBlocks lego_;  // Run in regular interval.
  Magnets mags_;     // Run in regular interval.
  Beat lego_beat_;
  Beat mags_beat_;
  const double gr_ = 21.0;  // delta_rotor = delta_output * gear_ratio
  const PmCmd* const pm_cmd_template_;

  ////////////////////
  // Configuration: //

  struct Configuration {
    struct {
      int id_l = 1, id_r = 2;
      uint8_t bus = 1;
    } servo;
    struct {
      double c, x_c, y_c;
    } lps;
    struct {
      int pin_l = 23, pin_r = 29;
      uint32_t run_interval = 20;
    } lego;
    struct {
      uint8_t pin_la = 3, pin_lt = 4, pin_ra = 5, pin_rt = 6;
      uint32_t run_interval = 100;
    } mags;
  };

  //////////////////
  // Constructor: //

  Basilisk(const Configuration& cfg)
      : l_{cfg.servo.id_l, cfg.servo.bus, &globals::pm_fmt, &globals::q_fmt},
        r_{cfg.servo.id_r, cfg.servo.bus, &globals::pm_fmt, &globals::q_fmt},
        lr_{&l_, &r_},
        pm_cmd_template_{&globals::pm_cmd_template},
        lps_{cfg.lps.c, cfg.lps.x_c, cfg.lps.y_c},
        imu_{},
        lego_{cfg.lego.pin_l, cfg.lego.pin_r},
        mags_{&lego_,                            //
              cfg.mags.pin_la, cfg.mags.pin_lt,  //
              cfg.mags.pin_ra, cfg.mags.pin_rt},
        lego_beat_{cfg.lego.run_interval},
        mags_beat_{cfg.mags.run_interval} {}

  ///////////////////
  // Setup method: //

  // Should be called before use.
  bool Setup() {
    if (!CanFdDriverInitializer::Setup(1)) {
      Serial.println("Basilisk: CanFdDriver setup failed");
      return false;
    }
    CommandBoth([](Servo* s) {
      s->SetStop();
      s->Print();
    });
    Serial.println("Basilisk: Both Servos Stopped, Queried and Printed");
    if (!lps_.Setup()) {
      Serial.println("Basilisk: LPS setup failed");
      return false;
    }
    if (!imu_.Setup()) {
      Serial.println("Basilisk: IMU setup failed");
      return false;
    }
    if (!lego_.Setup()) {
      Serial.println("Basilisk: LegoBlocks setup failed");
      return false;
    }
    if (!mags_.Setup()) {
      Serial.println("Basilisk: Magnets setup failed");
      return false;
    }

    Serial.println("Basilisk: All components setup succeeded");
    return true;
  }

  ////////////////////////
  // Components runner: //

  void Run() {
    lps_.Run();
    imu_.Run();
    if (lego_beat_.Hit()) lego_.Run();
    if (mags_beat_.Hit()) mags_.Run();
  }

  //////////////////////////////
  // Basilisk Command struct: //

  enum class MuxCR : bool { Xbee, Neokey } mux_cr_ = MuxCR::Neokey;

  struct Command {
    uint8_t oneshots;  // bit 0: SetBaseYaw

    enum class Mode {
      Idle_Init,     // -> Idle_Nop
      Idle_Nop,      // .
      Wait,          // -> Exit
      Free,          // -> Wait -> Idle
      SetMags,       // -> Wait -> Exit
      SetPhis_Init,  // -> Wait -> SetPhis_Stop
      SetPhis_Stop,  // -> Exit
      Pivot_Init,    // -> SetMags -> SetPhis -> Pivot_Kick  // Set didimbal.
      Pivot_Kick,    // -> SetMags -> SetPhis -> Exit
      Walk,          // -> Pivot -> Walk(++step) ~> Idle
      ///////////////////////////////////////////////////////////////////
      // Diamond_Init,  // -> SetPhis -> Diamond_Step
      // Diamond_Step,  // -> SetPhis -> Diamond_Step(++step) ~> Idle_Init
      // Gee_Init,      // -> SetPhis -> Gee_Step
      // Gee_Step,      // -> SetPhis -> Gee_Step(++step) ~> Idle_Init
      // Spin,          // -> Face -> Spin(++step) ~> Idle_Init
      // WalkToDir,     // -> SetMags -> SetPhis -> WalkToDir(++step) ~>
      // Idle_Init WalkToPos,     // -> SetMags -> SetPhis -> WalkToPos(++step)
      // ~> Idle_Init BounceWalk,    // . Orbit          // .
    } mode = Mode::Idle_Init;

    struct Wait {
      Mode exit_to_mode;
      bool (*exit_condition)(Basilisk*);
      uint32_t init_time;
    } wait;

    struct SetMags {
      Mode exit_to_mode;
      MagnetStrength strengths[4];
      bool expected_contact[2];  // true: contact, false: detachment
      N64 verif_thr;
      uint32_t min_wait_time;
      uint32_t max_wait_time;
    } set_mags;

    struct SetPhis {
      Mode exit_to_mode;
      Phi tgt_phi[2];          // [0]: l, [1]: r
      PhiDot tgt_phidot[2];    // [0]: l, [1]: r
      PhiDDot tgt_phiddot[2];  // [0]: l, [1]: r
      double damp_thr = 0.1;
      double fix_thr = 0.01;
    } set_phis;

    struct Pivot {
      Mode exit_to_mode;
      double tgt_yaw;  // NaN means current yaw.
      Phi stride;      // Forward this much more from tgt_yaw.
                       // Negative value manifests as walking backwards.
      Phi bend[2];     // = tgt_sig - tgt_yaw. NaN possible.
      LR didimbal;     // Foot to pivot about.
    } pivot;

    struct Walk {
      Pivot pivot[2];
      uint8_t tot_steps;  // Counting both left and right steps.
      LR init_didimbal;

     private:
      friend struct ModeRunners;
      uint8_t cur_step;
      LR cur_didimbal;
    } walk;

    // struct Diamond {
    //   friend struct ModeRunners;
    //   Diamond() {}
    //   Diamond(const double& _stride) : stride{_stride} {}
    //   // Half of top-bottom angle of the diamond.
    //   double stride = 0.125;
    //  private:
    //   uint8_t current_step = 0;
    //   // phi_l and phi_r for Step 0, 1, 2, 3.
    //   double tgt_phi(uint8_t step) {
    //     switch (step) {
    //       case 0:
    //         return stride;
    //       case 1:
    //         return -0.5 - stride;
    //       case 2:
    //         return -0.5 + stride;
    //       case 3:
    //         return -stride;
    //       default:
    //         return stride;
    //     }
    //   }
    // } diamond;
    // class Gee {
    //   friend struct ModeRunners;
    //  public:
    //   Gee() {}
    //   Gee(const double& _stride, const uint8_t& _steps)
    //       : stride{_stride}, steps{_steps} {}
    //   // Delta sigma between zero pose and shear pose.
    //   // Negative value manifests as moving left, and positive right.
    //   double stride = 0.125;
    //   // Total steps counting both ankle shears and toe shears.
    //   uint8_t steps = 8;
    //   // false = fix ankle, true = fix toe.
    //   bool phase = false;
    //  private:
    //   uint8_t current_step = 0;
    // } gee;
  } cmd_;

  struct Reply {
    double* lpsx;
    double* lpsy;
    double* yaw;
  } rpl_;

  ///////////////////
  // Util methods: //

  template <typename ServoCommand>
  void CommandBoth(ServoCommand c) {
    for (auto* s : lr_) c(s);
  }

  void Print() {
    CommandBoth([](Servo* s) {
      s->SetQuery();
      s->Print();
    });
  }
};
