#pragma once

#include "../components/neokey.h"
#include "../servo_units/basilisk.h"

class NeokeyCommandReceiver {
 public:
  NeokeyCommandReceiver(Neokey& nk) : nk_{nk} {}

  void (*callback_)(uint16_t key) = [](uint16_t key) {
    nk_cmd_ = key + 1;
    b_->crmux_ = Basilisk::CRMux::Neokey;

    // Serial.print("NeokeyCommandReceiver: Key rose: ");
    // Serial.print(key);
    // Serial.print(", nk_cmd_: ");
    // Serial.println(nk_cmd_);
  };

  // Should be called before use.
  bool Setup(Basilisk* b) {
    if (!b) {
      Serial.println("NeokeyCommandReceiver: Null pointer to Basilisk");
      return false;
    }
    b_ = b;
    Serial.println("NeokeyCommandReceiver: Registered reference to Basilisk");

    if (!nk_.Setup(callback_)) {
      Serial.println("NeokeyCommandReceiver: Neokey setup failed");
      return false;
    };

    Serial.println("NeokeyCommandReceiver: Setup complete");
    return true;
  }

  // Should be called in regular interval short enough to ensure that
  // no physical press of a button is missed between.
  void Run() { nk_.Run(); }

  static void Parse() {
    using C = Basilisk::Command;
    using M = C::Mode;
    static auto& c = b_->cmd_;
    static auto& m = c.mode;

    switch (nk_cmd_) {
      case 0: {  // Last Neokey Command already processed, so don't touch.
        return;
      } break;
      case 1: {
        m = M::Idle_Init;
      } break;
      case 2: {
        m = M::Free;
      } break;
      case 3: {
        b_->cmd_.oneshots |= (1 << 1);
      } break;
      case 4: {
        m = M::Pivot_Init;
        c.pivot.didimbal = BOOL_L;
        c.pivot.tgt_yaw = [](Basilisk*) { return 0.0; };
        c.pivot.stride = 0.125;
        c.pivot.bend[0] = 0.0;
        c.pivot.bend[1] = 0.0;
        c.pivot.speed = 0.1;
        c.pivot.acclim = 1.0;
        c.pivot.min_dur = -1;
        c.pivot.max_dur = -1;
        c.pivot.exit_to_mode = M::Idle_Init;
      } break;
      case 5: {
        m = M::Pivot_Init;
        c.pivot.didimbal = BOOL_R;
        c.pivot.tgt_yaw = [](Basilisk*) { return 0.0; };
        c.pivot.stride = 0.125;
        c.pivot.bend[0] = 0.0;
        c.pivot.bend[1] = 0.0;
        c.pivot.speed = 0.1;
        c.pivot.acclim = 1.0;
        c.pivot.min_dur = 0;
        c.pivot.max_dur = -1;
        c.pivot.exit_to_mode = M::Idle_Init;
      } break;
      case 6: {
        m = M::Pivot_Init;
        c.pivot.didimbal = BOOL_L;
        c.pivot.tgt_yaw = [](Basilisk*) { return 0.0; };
        c.pivot.stride = -0.125;
        c.pivot.bend[0] = 0.0;
        c.pivot.bend[1] = 0.0;
        c.pivot.speed = 0.1;
        c.pivot.acclim = 1.0;
        c.pivot.min_dur = 0;
        c.pivot.max_dur = -1;
        c.pivot.exit_to_mode = M::Idle_Init;
      } break;
      case 7: {
        m = M::Pivot_Init;
        c.pivot.didimbal = BOOL_R;
        c.pivot.tgt_yaw = [](Basilisk*) { return 0.0; };
        c.pivot.stride = -0.125;
        c.pivot.bend[0] = 0.0;
        c.pivot.bend[1] = 0.0;
        c.pivot.speed = 0.1;
        c.pivot.acclim = 1.0;
        c.pivot.min_dur = 0;
        c.pivot.max_dur = -1;
        c.pivot.exit_to_mode = M::Idle_Init;
      } break;
      case 8: {
        m = M::WalkToDir;
        c.walk_to_dir.init_didimbal = BOOL_L;
        c.walk_to_dir.tgt_yaw = 0.0;
        c.walk_to_dir.stride = 0.05;
        c.walk_to_dir.bend[IDX_L] = 0.0;
        c.walk_to_dir.bend[IDX_R] = 0.0;
        c.walk_to_dir.speed = 0.1;
        c.walk_to_dir.acclim = 1.0;
        c.walk_to_dir.min_stepdur = 1000;
        c.walk_to_dir.max_stepdur = 10000;
        c.walk_to_dir.steps = 16;
      } break;
      case 9: {
        m = M::WalkToDir;
        c.walk_to_dir.init_didimbal = BOOL_L;
        c.walk_to_dir.tgt_yaw = 0.5;
        c.walk_to_dir.stride = 0.05;
        c.walk_to_dir.bend[IDX_L] = 0.0;
        c.walk_to_dir.bend[IDX_R] = 0.0;
        c.walk_to_dir.speed = 0.1;
        c.walk_to_dir.acclim = 1.0;
        c.walk_to_dir.steps = 8;
        c.walk_to_dir.min_stepdur = 1000;
        c.walk_to_dir.max_stepdur = 10000;
        c.walk_to_dir.interval = 0;
      } break;
      case 10: {
        m = M::Sufi;
        c.sufi.init_didimbal = BOOL_L;
        c.sufi.dest_yaw = 0.5;
        c.sufi.stop_thr = 0.02;
        c.sufi.stride = 0.05;
        c.sufi.bend[IDX_L] = 0.0;
        c.sufi.bend[IDX_R] = 0.0;
        c.sufi.speed = 0.1;
        c.sufi.acclim = 1.0;
        c.sufi.steps = 16;
        c.sufi.min_stepdur = 2000;
        c.sufi.max_stepdur = 8000;
        c.sufi.interval = 200;
      } break;
      case 11: {
        m = M::SetPhis_Init;
        c.set_phis.tgt_phi[IDX_L] = NaN;
        c.set_phis.tgt_phi[IDX_R] = 0.0;
        c.set_phis.tgt_phispeed[IDX_L] = 0.1;
        c.set_phis.tgt_phispeed[IDX_R] = 0.1;
        c.set_phis.tgt_phiacclim[IDX_L] = 1.0;
        c.set_phis.tgt_phiacclim[IDX_R] = 1.0;
        c.set_phis.damp_thr = 0.05;
        c.set_phis.stop_thr = 0.01;
        c.set_phis.min_dur = 2000;
        c.set_phis.max_dur = 100000;
        c.set_phis.exit_condition = [](Basilisk*) { return false; };
      } break;
      case 12: {
        m = M::SetPhis_Init;
        c.set_phis.tgt_phi[IDX_L] = 0.0;
        c.set_phis.tgt_phi[IDX_R] = NaN;
        c.set_phis.tgt_phispeed[IDX_L] = 0.1;
        c.set_phis.tgt_phispeed[IDX_R] = 0.1;
        c.set_phis.tgt_phiacclim[IDX_L] = 1.0;
        c.set_phis.tgt_phiacclim[IDX_R] = 1.0;
        c.set_phis.damp_thr = 0.05;
        c.set_phis.stop_thr = 0.01;
        c.set_phis.min_dur = 2000;
        c.set_phis.max_dur = 100000;
        c.set_phis.exit_condition = [](Basilisk*) { return false; };
      } break;

      // case 4: {  // CatWalk
      //   m = M::Walk_Init;
      //   c.walk = C::WalkStraight{0.25, 0.0, 0.0, 4, false};
      // } break;
      // case 5: {  // BabyWalk
      //   m = M::Walk_Init;
      //   c.walk = C::WalkStraight{10.0 / 360.0, 0.0, 0.0, 16, false};
      // } break;
      // case 6: {  // EightWalk
      //   m = M::Walk_Init;
      //   c.walk = C::WalkStraight{0.125, 0.125, -0.125, 8, false};
      // } break;
      // case 7: {  // SquareDiamond
      //   m = M::Diamond_Init;
      //   c.diamond = C::Diamond{0.125};
      // } break;
      // case 8: {  // LeftGee
      //   m = M::Gee_Init;
      //   c.gee = C::Gee{-0.125, 8};
      // } break;
      // case 9: {  // RightGee
      //   m = M::Gee_Init;
      //   c.gee = C::Gee{-0.125, 8};
      // } break;
      default: {  // Whatever left keys are assigned Idle Mode.
        m = M::Idle_Init;
      } break;
    }
  }

  inline static uint16_t nk_cmd_ = 0;

 private:
  Neokey& nk_;
  inline static Basilisk* b_ = nullptr;
};
