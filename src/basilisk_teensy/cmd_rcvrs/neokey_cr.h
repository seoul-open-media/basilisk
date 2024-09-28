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
        c.oneshots |= (1 << 1);  // SetBaseYaw
      } break;
        // case 4: {
        //   m = M::PivSpin;
        //   c.piv_spin.didimbal = BOOL_L;
        //   c.piv_spin.dest_yaw = NaN;
        //   c.piv_spin.exit_thr = NaN;
        //   c.piv_spin.stride = -0.1;
        //   c.piv_spin.bend[IDX_L] = 0.0;
        //   c.piv_spin.bend[IDX_R] = 0.0;
        //   c.piv_spin.speed = 0.1;
        //   c.piv_spin.acclim = 1.0;
        //   c.piv_spin.min_stepdur = 0;
        //   c.piv_spin.max_stepdur = -1;
        //   c.piv_spin.interval = 0;
        //   c.piv_spin.steps = -1;
        // } break;
        // case 4: {
        //   m = M::Pivot_Init;
        //   c.pivot.didimbal = BOOL_L;
        //   c.pivot.tgt_yaw = [](Basilisk*) { return NaN; };
        //   c.pivot.stride = 0.125;
        //   c.pivot.bend[IDX_L] = 0.0;
        //   c.pivot.bend[IDX_R] = 0.0;
        //   c.pivot.speed = 0.1;
        //   c.pivot.acclim = 1.0;
        //   c.pivot.min_dur = 0;
        //   c.pivot.max_dur = -1;
        //   c.pivot.exit_to_mode = M::Idle_Init;
        // } break;
        // case 4: {
        //   m = M::Pivot_Init;
        //   c.pivot.didimbal = BOOL_R;
        //   c.pivot.tgt_yaw = [](Basilisk*) { return 0.0; };
        //   c.pivot.stride = 0.125;
        //   c.pivot.bend[IDX_L] = 0.0;
        //   c.pivot.bend[IDX_R] = 0.0;
        //   c.pivot.speed = 0.1;
        //   c.pivot.acclim = 1.0;
        //   c.pivot.min_dur = 0;
        //   c.pivot.max_dur = -1;
        //   c.pivot.exit_to_mode = M::Idle_Init;
        // } break;
        // case 6: {
        //   m = M::Pivot_Init;
        //   c.pivot.didimbal = BOOL_L;
        //   c.pivot.tgt_yaw = [](Basilisk*) { return 0.0; };
        //   c.pivot.stride = -0.125;
        //   c.pivot.bend[0] = 0.0;
        //   c.pivot.bend[1] = 0.0;
        //   c.pivot.speed = 0.15;
        //   c.pivot.acclim = 1.0;
        //   c.pivot.min_dur = 0;
        //   c.pivot.max_dur = -1;
        //   c.pivot.exit_to_mode = M::Idle_Init;
        // } break;
        // case 7: {
        //   m = M::Pivot_Init;
        //   c.pivot.didimbal = BOOL_R;
        //   c.pivot.tgt_yaw = [](Basilisk*) { return 0.0; };
        //   c.pivot.stride = -0.125;
        //   c.pivot.bend[0] = 0.0;
        //   c.pivot.bend[1] = 0.0;
        //   c.pivot.speed = 0.15;
        //   c.pivot.acclim = 1.0;
        //   c.pivot.min_dur = 0;
        //   c.pivot.max_dur = -1;
        //   c.pivot.exit_to_mode = M::Idle_Init;
        // } break;
        // case 4: {
        //   m = M::WalkToDir;
        //   c.walk_to_dir.init_didimbal = BOOL_L;
        //   c.walk_to_dir.tgt_yaw = 0.0;
        //   c.walk_to_dir.stride = 0.1;
        //   c.walk_to_dir.bend[IDX_L] = 0.0;
        //   c.walk_to_dir.bend[IDX_R] = 0.0;
        //   c.walk_to_dir.speed = 0.1;
        //   c.walk_to_dir.acclim = 1.0;
        //   c.walk_to_dir.min_stepdur = 1000;
        //   c.walk_to_dir.max_stepdur = 3000;
        //   c.walk_to_dir.interval = 0;
        //   c.walk_to_dir.steps = -1;
        // } break;
        // case 9: {
        //   m = M::WalkToDir;
        //   c.walk_to_dir.init_didimbal = BOOL_L;
        //   c.walk_to_dir.tgt_yaw = 0.5;
        //   c.walk_to_dir.stride = 0.05;
        //   c.walk_to_dir.bend[IDX_L] = 0.0;
        //   c.walk_to_dir.bend[IDX_R] = 0.0;
        //   c.walk_to_dir.speed = 0.15;
        //   c.walk_to_dir.acclim = 1.0;
        //   c.walk_to_dir.min_stepdur = 1000;
        //   c.walk_to_dir.max_stepdur = 3000;
        //   c.walk_to_dir.interval = 0;
        //   c.walk_to_dir.steps = -1;
        // } break;
      case 4: {
        m = M::WalkToPos;
        c.walk_to_pos.init_didimbal = BOOL_L;
        c.walk_to_pos.tgt_pos = Vec2{100, 100};
        c.walk_to_pos.dist_thr = 35;
        c.walk_to_pos.stride = 0.1;
        c.walk_to_pos.bend[IDX_L] = 0.0;
        c.walk_to_pos.bend[IDX_R] = 0.0;
        c.walk_to_pos.speed = 0.1;
        c.walk_to_pos.acclim = 1.0;
        c.walk_to_pos.min_stepdur = 1000;
        c.walk_to_pos.max_stepdur = 3000;
        c.walk_to_pos.interval = 0;
        c.walk_to_pos.steps = -1;
      } break;
      // case 4: {
      //   m = M::Sufi;
      //   c.sufi.init_didimbal = BOOL_L;
      //   c.sufi.dest_yaw = NaN;
      //   c.sufi.exit_thr = NaN;
      //   c.sufi.stride = 0.1;
      //   c.sufi.bend[IDX_L] = 0.0;
      //   c.sufi.bend[IDX_R] = 0.0;
      //   c.sufi.speed = 0.1;
      //   c.sufi.acclim = 1.0;
      //   c.sufi.min_stepdur = 1000;
      //   c.sufi.max_stepdur = 3000;
      //   c.sufi.interval = 0;
      //   c.sufi.steps = -1;
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
