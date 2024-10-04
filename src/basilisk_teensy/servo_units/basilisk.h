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
  //////////////////////
  // Configurations : //

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

  /////////////////
  // Components: //

  Servo l_, r_;
  Servo* s_[2] = {nullptr, nullptr};
  Lps lps_;          // Run every loop().
  Imu imu_;          // Run every loop().
  LegoBlocks lego_;  // Run in regular interval.
  Magnets mags_;     // Run in regular interval.

  //////////////////
  // Constructor: //

  Basilisk(const Configuration& cfg)
      : cfg_{cfg},
        pm_cmd_template_{&globals::pm_cmd_template},
        l_{cfg.servo.id_l, cfg.servo.bus, &globals::pm_fmt, &globals::q_fmt},
        r_{cfg.servo.id_r, cfg.servo.bus, &globals::pm_fmt, &globals::q_fmt},
        s_{&l_, &r_},
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
      s->SetQuery();
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

  enum class CRMux : bool { Xbee, Neokey } crmux_ = CRMux::Xbee;

  struct Command {
    uint8_t oneshots;  // bit 0: CRMuxXbee
                       // bit 1: SetBaseYaw
                       // bit 2: Inspire

    struct SetBaseYaw {
      double offset;
    } set_base_yaw;

    struct Inspire {
    } inspire;

    enum class Mode : uint8_t {
      // A child Mode cannot be future-chained after its parent Mode.
      // No loop should be formed in a future-chain.

      /* Idle: Kill everything. Relax.
       * - Stop both Servos, attach all magnets,
       * - then do nothing. */
      Idle_Init = 0,  // -> Idle_Nop
      Idle_Nop = 1,   // .

      /* Wait: Wait for some condition to be met.
       *       Future-chain-able.
       * - Loop until given condition is met,
       * - then exit to designated Mode. */
      Wait = 2,  // -> Exit

      /* Free: Stop and release magnets so you can lift it up from the ground.
       *       Magnets will be reactivated when 3 seconds has passed. Only for
       *       convenience in development or emergency during deployment.
       * - Stop both Servos, release all magnets,
       * - then wait 3 seconds,
       * - then exit to Idle Mode. */
      Free = 3,  // -> Wait -> Idle

      /* DoPreset: Do a preset. */
      DoPreset = 4,

      /* SetMags: Control magnets.
       *          Future-chain-able.
       * - Attach or release individual magnets,
       * - then wait for contact/detachment verification,
       * - then exit to designated Mode.
       * - Duration will be clamped. */
      SetMags_Init = 5,  // -> SetMags_Wait
      SetMags_Wait = 6,  // -> Exit

      /* RandomMags: Randomly tap-dance. */
      RandomMags_Init = 19,
      RandomMags_Do = 18,

      /* SetPhis: Control Servos to achieve target phis.
       *          Future-chain-able.
       * - PositionMode-Command Servos continuously with .position
       *   set to NaN, .velocity and .accel_limit set to computed as follows:
       *     tgt_rtrvel = tgt_phi == NaN || abs(tgt_delta_phi) < fix_thr ? 0 :
       *                  21 * tgt_phispeed * (tgt_delta_phi >  damp_thr ?  1 :
       *                                       tgt_delta_phi < -damp_thr ? -1 :
       *                                       tgt_delta_phi / damp_thr);
       *     tgt_rtracclim = 21 * tgt_phiacclim;
       *   Fix cycles count is incremented every cycle where tgt_rtrvel == 0
       *   and reset elsewhere. Wait until fix cycles count reaches threshold
       *   for both Servos,
       * - then exit to designated Mode.
       * - Phi and duration will be clamped throughout. */
      SetPhis_Init = 7,  // -> SetPhis_Move
      SetPhis_Move = 8,  // -> Exit

      /* Pivot: Pivot one foot (kickbal) about the other (didimbal).
       *        The single fundamental element of all Basilisk movement
       *        except Gee.
       *        Future-chain-able.
       *        IsigD + IphiD = Ipsi = IsigK + IphiK
       *     -> (Tpsi + bD) + ? = Ipsi = IsigK + IphiK  (set didimbal)
       *     -> (Tpsi + bD) + ? = Tpsi +- s = (Tpsi + bK) + ?  (kick)
       *
       * - Attach and fix kickbal, release didimbal, and set phi_didim,
       * - then attach didimbal, release kickbal and set both phis,
       * - then exit to designated Mode.
       * - Phi and duration will be clamped throughout. */
      Pivot_Init = 9,   // -> SetMags -> SetPhis -> Pivot_Kick
      Pivot_Kick = 10,  // -> SetMags -> SetPhis -> Exit

      /* PivSeq: Perform a series of Pivots with time intervals.
       *         Future-chain-able. */
      PivSeq_Init = 11,  // -> PivSeq_Step
      PivSeq_Step = 12,  // -> Pivot -> PivSeq_Step(++step) ~> Exit

      /* Walk: An instance of PivSeq implementing unipedalism. */
      PivSpin = 13,  // -> PivSeq -> Idle

      /* Walk: An instance of PivSeq implementing bipedalism. */
      Walk = 20,  // -> PivSeq -> Idle

      /* Walk Variants: Instances of Walk. */
      WalkToDir = 21,  // -> Walk -> Idle
      WalkToPos = 22,  // -> Walk -> Idle
      Sufi = 23,       // -> Walk -> Idle
      Orbit = 24,
      Diamond = 25,
      RandomWalk = 26,
      GhostWalk = 27,

      /* Gee: */
      Shear_Init = 250,
      Shear_Move = 251,
      Gee = 252,
    } mode = Mode::Idle_Init;

    struct DoPreset {
      uint16_t idx;
    } do_preset;

    struct Wait {
      uint32_t init_time;
      bool (*exit_condition)(Basilisk*);
      Mode exit_to_mode;
    } wait;

    struct SetMags {
      MagStren strengths[4];
      bool expected_state[2];     // [0]: l, [1]: r
                                  // true: contact, false: detachment
      N64 verif_thr;              // Exit condition priority:
      uint32_t min_dur, max_dur;  // max_dur > min_dur > lego_verification
      Mode exit_to_mode;
    } set_mags;

    struct RandomMags {
      uint32_t min_phase_dur, max_phase_dur;
      uint32_t dur;
    } random_mags;

    struct SetPhis {
      Phi tgt_phi[2];  // [0]: l, [1]: r
                       // NaN means fix phi (speed and acclim ignored).
      PhiSpeed tgt_phispeed[2];    // [0]: l, [1]: r
      PhiAccLim tgt_phiacclim[2];  // [0]: l, [1]: r
      PhiThr damp_thr;
      PhiThr fix_thr;
      uint8_t fixing_cycles_thr;          // Exit condition priority:
      uint32_t min_dur, max_dur;          // (max_dur || exit_condition)
      bool (*exit_condition)(Basilisk*);  // > (min_dur && fixed_enough)
      Mode exit_to_mode;
    } set_phis;

    struct Pivot {
      LR didimbal;                   // Foot to pivot about.
      double (*tgt_yaw)(Basilisk*);  // Evaluated at Pivot_Init
                                     // and used throughout Pivot.
                                     // NaN means yaw at Pivot_Init.
      // (*.*) oO(Ignore me...)
      double stride;  // Forward this much more from tgt_yaw.
                      // Negative value manifests as walking backwards.
                      // NaN means do NOT kick.
      Phi bend[2];    // [0]: l, [1]: r
                      // tgt_sig == tgt_yaw + bend
                      // or bend == -tgt_phi (at stride 0)
                      // NaN means preserve initial sig for didimbal,
                      // initial phi for kickbal.
      PhiSpeed speed;
      PhiAccLim acclim;
      uint32_t min_dur, max_dur;
      bool (*exit_condition)(Basilisk*);  // Passed down to SetPhis.
                                          // Exit condition priority:
                                          // max_dur > exit_condition > min_dur
      Mode exit_to_mode;
    } pivot;

    struct PivSeq {
      Pivot (*pivots)(Basilisk*, int);  // exit_to_mode will be
                                        // overwritten by PivSeq.
      uint32_t (*intervals)(Basilisk*, int);
      int steps;                          // Counting both feet.
      bool (*exit_condition)(Basilisk*);  // This is exit condition
                                          // evaluated every interval
                                          // between Pivots. Exit condition
                                          // while Pivoting should be set
                                          // at Pivot::exit_condition.
                                          // Exit condition priority:
                                          // exit_condition > steps
      Mode exit_to_mode;
    } pivseq;

    struct Walk {
      LR init_didimbal;
      double (*tgt_yaw[2])(Basilisk*);          // [0]: l, [1]: r (didimbal)
      double (*stride[2])(Basilisk*);           // [0]: l, [1]: r (didimbal)
      Phi bend[2];                              // [0]: l, [1]: r (didimbal)
      PhiSpeed speed[2];                        // [0]: l, [1]: r (didimbal)
      PhiAccLim acclim[2];                      // [0]: l, [1]: r (didimbal)
      uint32_t min_stepdur[2], max_stepdur[2];  // [0]: l, [1]: r (didimbal)
      uint32_t interval[2];                     // [0]: l, [1]: r (didimbal)
      uint8_t steps;                            // Counting both feet.
      bool (*exit_condition)(Basilisk*);  // Passed down to PivSeq AND Pivot.
    } walk;

    struct WalkToDir {
      LR init_didimbal;
      double tgt_yaw;  // NaN means yaw at WalkToDir initialization.
      double stride;
      Phi bend[2];
      PhiSpeed speed;
      PhiAccLim acclim;
      uint32_t min_stepdur, max_stepdur;
      uint32_t interval;
      uint8_t steps;
    } walk_to_dir;

    struct WalkToPos {
      LR init_didimbal;
      Vec2 tgt_pos;
      double dist_thr;
      double stride;
      Phi bend[2];
      PhiSpeed speed;
      PhiAccLim acclim;
      uint32_t min_stepdur, max_stepdur;
      uint32_t interval;
      uint8_t steps;
    } walk_to_pos;

    struct Sufi {
      LR init_didimbal;
      double dest_yaw;  // NaN means no destination.
      double exit_thr;
      double stride;
      Phi bend[2];
      PhiSpeed speed;
      PhiAccLim acclim;
      uint32_t min_stepdur, max_stepdur;
      uint32_t interval;
      uint8_t steps;
    } sufi;

    struct PivSpin {
      LR didimbal;
      double dest_yaw;  // NaN means no destination.
      double exit_thr;
      double stride;
      Phi bend[2];
      PhiSpeed speed;
      PhiAccLim acclim;
      uint32_t min_stepdur, max_stepdur;
      uint32_t interval;
      uint8_t steps;
    } piv_spin;

    struct Orbit {
      Vec2 center;
      double tick;
    } orbit;

    struct Diamond {
      LR init_didimbal;
      double init_stride;
      PhiSpeed speed;
      PhiAccLim acclim;
      uint32_t min_stepdur, max_stepdur;
      uint32_t interval;
      uint8_t steps;
    } diamond;

    struct GhostWalk {
    } ghost_walk;

    struct Shear {
      AnkToe fix_which;
      Phi tgt_phi;
    };

    struct Gee {
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
      // false = attach ankle, true = attach toe.
      bool phase = false;

     private:
      uint8_t current_step = 0;
    } gee;
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
