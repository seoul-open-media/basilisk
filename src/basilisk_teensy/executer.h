#pragma once

#include "cmd_rcvrs/neokey_cr.h"
#include "cmd_rcvrs/xbee_cr.h"
#include "mode_runners/matome.h"
#include "oneshots/matome.h"

class Executer {
 public:
  Executer(Basilisk* b) : b_{b} {}

  void Run() {
    if (!XbeeCommandReceiver::receiving_ &&
        (XbeeCommandReceiver::xbee_cmd_.decoded.oneshots & 1 ||
         (XbeeCommandReceiver::xbee_cmd_.decoded.mode ==
              static_cast<uint8_t>(Basilisk::Command::Mode::DoPreset) &&
          XbeeCommandReceiver::xbee_cmd_.decoded.u.do_preset
                  .idx[b_->cfg_.suid] == 50002))) {
      b_->cmd_.oneshots |= 1;
    }

    BasiliskOneshots::Shoot(b_);

    for (uint8_t id = 0; id < 4; id++) {
      if (b_->mags_.heavenfall_warning_[id]) {
        b_->cmd_.mode = Basilisk::Command::Mode::Idle_Init;
        return;
      }
    }

    b_->CommandBoth([](Servo* s) { s->SetQuery(); });

    switch (b_->crmux_) {
      case Basilisk::CRMux::Neokey: {
        if (NeokeyCommandReceiver::nk_cmd_ != 0) {
          NeokeyCommandReceiver::Parse();
          NeokeyCommandReceiver::nk_cmd_ = 0;
        }
      } break;
      case Basilisk::CRMux::Xbee: {
        if (XbeeCommandReceiver::waiting_parse_) {
          XbeeCommandReceiver::Parse();
          XbeeCommandReceiver::waiting_parse_ = false;
        }
      } break;
      default:
        break;
    }

#if I_WANT_DEBUG
      // Serial.print("Mode ");
      // Serial.println(static_cast<uint8_t>(b_->cmd_.mode));
#endif

    auto* maybe_mode_runner = SafeAt(ModeRunners::mode_runners, b_->cmd_.mode);
    if (maybe_mode_runner) {
      (*maybe_mode_runner)(b_);
    } else {
      Serial.println("Mode NOT registered to ModeRunners::mode_runners");
    }
  }

 private:
  Basilisk* b_;
};
