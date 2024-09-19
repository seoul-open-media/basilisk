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
  Servo* s_[2] = {nullptr, nullptr};
  Lps lps_;          // Run every loop().
  Imu imu_;          // Run every loop().
  LegoBlocks lego_;  // Run in regular interval.
  Magnets mags_;     // Run in regular interval.

  ///////////////////////////////////
  // Configurations & constructor: //

  struct Configuration {
    uint8_t suid;  // 1 <= ID of this Basilisk <= 13
    struct {
      int id_l = 1, id_r = 2;
      uint8_t bus = 1;
    } servo;
    struct {
      double c, x_c, y_c;
      double minx, maxx, miny, maxy;
    } lps;
    struct {
      int pin_l = 23, pin_r = 29;
      uint32_t run_interval = 10;
    } lego;
    struct {
      uint8_t pin_la = 3, pin_lt = 4, pin_ra = 5, pin_rt = 6;
      uint32_t run_interval = 100;
    } mags;
  } cfg_;

  const double gr_ = 21.0;  // delta_rotor = delta_output * gear_ratio
  const PmCmd* const pm_cmd_template_;

  Basilisk(const Configuration& cfg)
      : cfg_{cfg},
        l_{cfg.servo.id_l, cfg.servo.bus, &globals::pm_fmt, &globals::q_fmt},
        r_{cfg.servo.id_r, cfg.servo.bus, &globals::pm_fmt, &globals::q_fmt},
        s_{&l_, &r_},
        pm_cmd_template_{&globals::pm_cmd_template},
        lps_{cfg.lps.c,    cfg.lps.x_c,  cfg.lps.y_c,  //
             cfg.lps.minx, cfg.lps.maxx, cfg.lps.miny, cfg.lps.maxy},
        imu_{},
        lego_{cfg.lego.pin_l, cfg.lego.pin_r},
        mags_{&lego_,                            //
              cfg.mags.pin_la, cfg.mags.pin_lt,  //
              cfg.mags.pin_ra, cfg.mags.pin_rt} {}

  ////////////////////////////////////////////////////////////
  // Setup method (should be called in setup() before use): //

  bool Setup() {
    if (!CanFdDriverInitializer::Setup(cfg_.servo.bus)) {
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
    static Beat lego_beat_{cfg_.lego.run_interval};
    if (lego_beat_.Hit()) lego_.Run();
    static Beat mags_beat_{cfg_.mags.run_interval};
    if (mags_beat_.Hit()) mags_.Run();
  }

  //////////////////////////////
  // Basilisk Command struct: //

  enum class CRMux : bool { Xbee, Neokey } crmux_ = CRMux::Neokey;

  struct Command {
    uint8_t oneshots;  // bit 0: CRMuxXbee
                       // bit 1: SetBaseYaw

    enum class Mode : uint8_t {
      // A child Mode cannot be future-chained after its parent Mode.
      // No loop should be formed in a future-chain.

      DoPreset,

      /* Idle: Kill everything. Relax.
       * - Stop both Servos, attach all magnets,
       * - then do nothing. */
      Idle_Init,  // -> Idle_Nop
      Idle_Nop,   // .

      /* Wait: Wait for some condition to be met.
       *       Future-chain-able.
       * - Loop until given condition is met,
       * - then exit to designated Mode. */
      Wait,  // -> Exit

      /* Free: Stop and release magnets so you can lift it up from the ground.
       *       Magnets will be reactivated when 3 seconds has passed. Only for
       *       convenience in development or emergency during deployment.
       * - Stop both Servos, release all magnets,
       * - then wait 3 seconds,
       * - then exit to Idle Mode. */
      Free,  // -> Wait -> Idle

      /* SetMags: Control magnets.
       *          Future-chain-able.
       * - Attach or release individual magnets,
       * - then wait for contact/detachment verification,
       * - then exit to designated Mode.
       * - Duration will be clamped. */
      SetMags_Init,  // -> SetMags_Wait
      SetMags_Wait,  // -> Exit

      /* SetPhis: Control Servos to achieve target phis.
       *          Future-chain-able.
       * - Reset fix cycles count,
       * - then PositionMode-Command Servos continuously with .position
       *   set to NaN, .velocity and .accel_limit set to computed as follows:
       *   tgt_rtrvel = tgt_phi == NaN || abs(tgt_delta_phi) < fix_thr ? 0 :
       *                21 * tgt_phispeed * (tgt_delta_phi >  damp_thr ?  1 :
       *                                     tgt_delta_phi < -damp_thr ? -1 :
       *                                     tgt_delta_phi / damp_thr);
       *   tgt_rtracclim = 21 * tgt_phiacclim;
       *   Fix cycles count is incremented every cycle where tgt_rtrvel == 0
       *   and reset elsewhere. Wait until fix cycles count reaches threshold
       *   for both Servos,
       * - then exit to designated Mode.
       * - Phi and duration will be clamped throughout. */
      SetPhis_Init,  // -> SetPhis_Move
      SetPhis_Move,  // -> Exit

      /* Pivot: Pivot one foot (kickbal) about the other (didimbal).
       *        The single fundamental element of all Basilisk movement except
       *        for Gee.
       *        Future-chain-able.
       *        IsigD + IphiD = Ipsi = IsigK + IphiK
       *     -> (Tpsi + bD) + ? = Ipsi = IsigK + IphiK  (set didimbal)
       *     -> (Tpsi + bD) + ? = Tpsi +- s = (Tpsi + bK) + ?  (kick)
       *
       * - Attach and fix kickbal, release didimbal, and set
       *   phi_didim = init_yaw - tgt_yaw - bend_didim
       *   so that sig_didim = init_yaw - phi_didim
       *                     = tgt_yaw + bend_didim,
       * - then attach didimbal, release kickbal and set
       *   phi_didim = -bend_didim +- stride
       *   phi_kick  = -bend_kick  +- stride
       *   so that yaw = sig_didim + phi_didim
       *               = tgt_yaw +- stride,
       *   and sig_kick = yaw - phi_kick
       *                = tgt_yaw + bend_kick,
       * - then exit to designated Mode.
       * - Phi and duration will be clamped throughout. */
      Pivot_Init,  // -> SetMags -> SetPhis -> Pivot_Kick
      Pivot_Kick,  // -> SetMags -> SetPhis -> Exit

      /* PivSeq: Perform a series of Pivots with time intervals.
       *         Future-chain-able. */
      PivSeq_Init,  // -> PivSeq_Step
      PivSeq_Step,  // -> Pivot -> PivSeq_Step(++step) ~> Exit

      /* Walk variants: Instances of PivSeq. */
      WalkToDir,  // -> PivSeq -> Idle
      WalkToPos,  // -> PivSeq -> Idle
      Sufi,

      Orbit,
      BounceWalk,
      RandomWalk,

      Diamond_Init,
      Diamond_Step,

      /* Gee: */
      Shear_Init,
      Shear_Move,
      Gee,
    } mode = Mode::Idle_Init;

    struct DoPreset {
      uint16_t idx;
    } do_preset;

    struct Wait {
      Mode exit_to_mode;
      bool (*exit_condition)(Basilisk*);
      uint32_t init_time;
    } wait;

    struct SetMags {
      Mode exit_to_mode;
      MagnetStrength strengths[4];
      bool expected_state[2];     // [0]: l, [1]: r
                                  // true: contact, false: detachment
      N64 verif_thr;              // Exit condition priority:
      uint32_t min_dur, max_dur;  // max_dur > min_dur > lego_verification

     private:
      friend struct ModeRunners;
      uint32_t init_time;
    } set_mags;

    struct SetPhis {
      Mode exit_to_mode;
      Phi tgt_phi[2];             // [0]: l, [1]: r
      PhiSpeed tgt_phispeed[2];   // [0]: l, [1]: r
      PhiAccel tgt_phiacclim[2];  // [0]: l, [1]: r
      PhiThreshold damp_thr;
      PhiThreshold fix_thr;
      uint8_t fix_cycles_thr;             // Exit condition priority:
      uint32_t min_dur, max_dur;          // max_dur > exit_condition
      bool (*exit_condition)(Basilisk*);  // > (min_dur && fixed_enough)

     private:
      friend struct ModeRunners;
      uint32_t init_time;
      uint8_t fix_cycles[2];
    } set_phis;

    struct Pivot {
      Mode exit_to_mode;
      LR didimbal;                   // Foot to pivot about.
      double (*tgt_yaw)(Basilisk*);  // Evaluated at Pivot_Init
                                     // and used throughout Pivot.
      // (*.*) oO(***)
      double stride;  // Forward this much more from tgt_yaw.
                      // Negative value manifests as walking backwards.
      Phi bend[2];    // [0]: l, [1]: r
                      // tgt_sig == tgt_yaw + bend
                      // or bend == -tgt_phi (at stride 0)
                      // NaN means preserve initial sig for didimbal,
                      // initial phi for kickbal.
      PhiSpeed speed;
      PhiAccel acclim;
      uint32_t min_dur, max_dur;          // Exit condition priority:
      bool (*exit_condition)(Basilisk*);  // max_dur > exit_condition > min_dur

     private:
      friend struct ModeRunners;
      uint32_t init_time;
    } pivot;

    struct PivSeq {
      Mode exit_to_mode;
      bool (*exit_condition)(Basilisk*);  // Evaluated every interval
                                          // between Pivots.
                                          // Exit condition priority:
                                          // exit_condition > steps

      // Pivot (*pivot_gtr)(Basilisk*, uint8_t);

      Pivot* pivots;  // exit_to_mode and minmax_dur will be written by PivSeq.
      uint32_t* min_durs;      // It is the PivSeq's clients responsibility to
      uint32_t* max_durs;      // cleanup memory.
      uint8_t size;            // Perform 0...(loop_begin_idx - 1) once, then
      uint8_t loop_begin_idx;  // perform (loop_begin_idx)...(size - 1) in loop,
      uint8_t steps;           // until steps steps are made in total.

     private:
      friend struct ModeRunners;
      uint8_t cur_step;
      uint8_t cur_idx;
    } pivseq;

    struct WalkToDir {
      LR init_didimbal;
      double tgt_yaw;  // NaN = initial yaw at WalkToDir_Init.
      double stride;
      Phi bend[2];
      PhiSpeed speed;
      PhiAccel acclim;
      uint32_t min_stepdur, max_stepdur;
      uint8_t steps;

     private:
      friend struct ModeRunners;
      Pivot pivots[2];
      uint32_t min_durs[2];
      uint32_t max_durs[2];
    } walk_to_dir;

    struct WalkToPos {
      LR init_didimbal;
      Vec2 tgt_pos;
      double prox_thr;
      double stride;
      Phi bend[2];
      PhiSpeed speed;
      PhiAccel acclim;
      uint32_t min_stepdur, max_stepdur;
      uint8_t steps;

     private:
      friend struct ModeRunners;
      Pivot pivots[2];
      uint32_t min_durs[2];
      uint32_t max_durs[2];
    } walk_to_pos;

    struct Sufi {
      LR init_didimbal;
      double dest_yaw;
      double stride;
      Phi bend[2];
      PhiSpeed speed;
      PhiAccel acclim;
      uint32_t min_stepdur, max_stepdur;
      uint8_t steps;

     private:
      friend struct ModeRunners;
      Pivot pivots[2];
      uint32_t min_durs[2];
      uint32_t max_durs[2];
    } sufi;

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
    //   // false = attach ankle, true = attach toe.
    //   bool phase = false;
    //  private:
    //   uint8_t current_step = 0;
    // } gee;
  } cmd_;

  struct Reply {
    Mode* mode;
    double* lpsx;
    double* lpsy;
    double* yaw;
  } rpl_;

  ///////////////////
  // Util methods: //

  template <typename ServoCommand>
  void CommandBoth(ServoCommand c) {
    for (auto* s : s_) c(s);
  }

  void Print() {
    CommandBoth([](Servo* s) {
      s->SetQuery();
      s->Print();
    });
  }
};
