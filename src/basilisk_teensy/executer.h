#pragma once

#include "cmd_rcvrs/neokey_cr.h"
#include "cmd_rcvrs/xbee_cr.h"
#include "mode_runners/mode_runners_matome.h"
#include "oneshots/oneshots_matome.h"

class Executer {
 public:
  Executer(Basilisk* b) : b_{b} {}

  void Run() {
    if (!XbeeCommandReceiver::receiving_ &&
        XbeeCommandReceiver::xbee_cmd_.decoded.oneshots & 1) {
      b_->cmd_.oneshots |= 1;
    }

    BasiliskOneshots::Shoot(b_);

    b_->CommandBoth([](Servo* s) { s->SetQuery(); });

    switch (b_->crmux_) {
      case Basilisk::CRMux::Neokey: {
        NeokeyCommandReceiver::Parse();
        NeokeyCommandReceiver::nk_cmd_ = 0;
      } break;
      case Basilisk::CRMux::Xbee: {
        XbeeCommandReceiver::Parse();
        XbeeCommandReceiver::waiting_parse_ = false;
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
