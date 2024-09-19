#pragma once

#include "cmd_rcvrs/neokey_cr.h"
#include "cmd_rcvrs/xbee_cr.h"
#include "mode_runners/mode_runners_matome.h"
#include "oneshots/oneshots_matome.h"

class Executer {
 public:
  Executer(Basilisk* b) : b_{b} {}

  void Run() {
    BasiliskOneshots::Shoot(b_);

    b_->CommandBoth([](Servo* s) { s->SetQuery(); });

    // if ()
    // {
    //   /* code */
    // }

    switch (b_->crmux_) {
      case Basilisk::CRMux::Neokey: {
        NeokeyCommandReceiver::Parse();
        NeokeyCommandReceiver::nk_cmd_ = 0;
      } break;
      case Basilisk::CRMux::Xbee: {
      } break;
      default:
        break;
    }

    auto* maybe_mode_runner = SafeAt(ModeRunners::mode_runners, b_->cmd_.mode);
    if (maybe_mode_runner) {
      (*maybe_mode_runner)(b_);
    } else {
      Serial.println("Mode NOT registered to ModeRunners::mode_runners");
      while (1);
    }
  }

 private:
  Basilisk* b_;
};
