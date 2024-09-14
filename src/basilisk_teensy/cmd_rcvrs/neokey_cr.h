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

    b_->mux_cr_ = Basilisk::MuxCR::Neokey;
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
      case 1: {  // Idle (stop Servos, fix magnets)
        m = M::Idle_Init;
      } break;
      case 2: {  // Free magnets
        m = M::Free;
      } break;
      case 3: {  // SquareWalk
        m = M::Walk_Init;
        c.walk = C::Walk{0.125, 0.0, 0.0, 8, false};
      } break;
      case 4: {  // CatWalk
        m = M::Walk_Init;
        c.walk = C::Walk{0.25, 0.0, 0.0, 4, false};
      } break;
      case 5: {  // BabyWalk
        m = M::Walk_Init;
        c.walk = C::Walk{10.0 / 360.0, 0.0, 0.0, 16, false};
      } break;
      case 6: {  // EightWalk
        m = M::Walk_Init;
        c.walk = C::Walk{0.125, 0.125, -0.125, 8, false};
      } break;
      case 7: {  // SquareDiamond
        m = M::Diamond_Init;
        c.diamond = C::Diamond{0.125};
      } break;
      case 8: {  // LeftGee
        m = M::Gee_Init;
        c.gee = C::Gee{-0.125, 8};
      } break;
      case 9: {  // RightGee
        m = M::Gee_Init;
        c.gee = C::Gee{-0.125, 8};
      } break;
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
