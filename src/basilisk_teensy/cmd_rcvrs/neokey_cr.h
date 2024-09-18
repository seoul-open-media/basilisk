#pragma once

#include "../components/neokey.h"
#include "../servo_units/basilisk.h"

class NeokeyCommandReceiver {
 public:
  NeokeyCommandReceiver(Neokey& nk) : nk_{nk} {}

  void (*callback_)(uint16_t key) = [](uint16_t key) {
    Serial.print("NeokeyCommandReceiver: Key rose: ");
    Serial.print(key);

    nk_cmd_ = key + 1;

    Serial.print(", nk_cmd_: ");
    Serial.println(nk_cmd_);

    b_->crmux_ = Basilisk::CRMux::Neokey;
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
  // no physical press of a button is missed.
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
        m = M::Pivot_Init;
        c.pivot.didimbal = BOOL_L;
        c.pivot.tgt_yaw = NaN;
        c.pivot.stride = 0.125;
        c.pivot.bend[0] = 0.0;
        c.pivot.bend[1] = 0.0;
        c.pivot.speed = 0.1;
        c.pivot.accel = 1.0;
        c.pivot.min_dur = 0;
        c.pivot.max_dur = 5000;
      } break;
      case 4: {
        m = M::WalkToDir;
        c.walk_to_dir.init_didimbal = BOOL_L;
        c.walk_to_dir.tgt_yaw = 0.0;
        c.walk_to_dir.stride = 0.0625;
        c.walk_to_dir.bend[IDX_L] = 0.0;
        c.walk_to_dir.bend[IDX_R] = 0.0;
        c.walk_to_dir.speed = 0.1;
        c.walk_to_dir.accel = 1.0;
        c.walk_to_dir.min_stepdur = 0;
        c.walk_to_dir.max_stepdur = 3000;
        c.walk_to_dir.steps = 6;
      } break;
      case 5: {
        m = M::PivSeq_Init;
        c.pivseq.exit_condition = [](Basilisk*) { return false; };
        void* memory = operator new(2 * sizeof(Basilisk::Command::Pivot));
        c.pivseq.pivots = static_cast<Basilisk::Command::Pivot*>(memory);
        for (int i = 0; i < 2; i++) {
          new (c.pivseq.pivots + i) Basilisk::Command::Pivot{};
        }
        c.pivseq.min_durs =
            static_cast<uint32_t*>(malloc(2 * sizeof(uint32_t)));
        c.pivseq.max_durs =
            static_cast<uint32_t*>(malloc(2 * sizeof(uint32_t)));

        c.pivseq.pivots[0].didimbal = BOOL_L;
        c.pivseq.pivots[0].tgt_yaw = 0.0;
        c.pivseq.pivots[0].stride = 0.125;
        c.pivseq.pivots[0].bend[0] = 0.0;
        c.pivseq.pivots[0].bend[1] = 0.0;
        c.pivseq.pivots[0].speed = 0.1;
        c.pivseq.pivots[0].accel = 1.0;
        c.pivseq.min_durs[0] = 0;
        c.pivseq.max_durs[0] = 5000;

        c.pivseq.pivots[1].didimbal = BOOL_R;
        c.pivseq.pivots[1].tgt_yaw = 0.0;
        c.pivseq.pivots[1].stride = 0.125;
        c.pivseq.pivots[1].bend[0] = 0.0;
        c.pivseq.pivots[1].bend[1] = 0.0;
        c.pivseq.pivots[1].speed = 0.1;
        c.pivseq.pivots[1].accel = 1.0;
        c.pivseq.min_durs[1] = 0;
        c.pivseq.max_durs[1] = 5000;

        c.pivseq.size = 2;
        c.pivseq.loop_begin_idx = 0;
        c.pivseq.steps = 2;
        c.pivseq.exit_to_mode = M::Idle_Init;
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
