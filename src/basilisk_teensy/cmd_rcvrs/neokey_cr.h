#pragma once

#include "../components/neokey.h"
#include "../servo_units/basilisk.h"
#include "../tests/modes.h"

class NeokeyCommandReceiver {
 public:
  NeokeyCommandReceiver(Neokey& nk) : nk_{nk} {}

  void (*callback_)(uint16_t key) = [](uint16_t key) {
    nk_cmd_ = key + 1;
    b_->crmux_ = Basilisk::CRMux::Neokey;

#if I_WANT_DEBUG
    Serial.print("NeokeyCommandReceiver: Key rose: ");
    Serial.print(key);
    Serial.print(", nk_cmd_: ");
    Serial.println(nk_cmd_);
#endif
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
    using M = Basilisk::Command::Mode;
    static auto& c = b_->cmd_;
    static auto& m = c.mode;

    switch (nk_cmd_) {
      case 0: {  // Last Neokey Command already processed, so don't touch.
      } break;
      case 1: {
        m = M::Idle_Init;
      } break;
      case 2: {
        m = M::Free;
      } break;
      case 3: {  // SetBaseYaw(0.0)
        c.oneshots |= (1 << 1);
        c.set_base_yaw.offset = 0.0;
      } break;
      case 4: {
        tests::Sufi(b_);
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
