#pragma once

#include "cmd_rcvrs/neokey_cr.h"
#include "mode_runners/mode_runners_matome.h"

class Executer {
 public:
  Executer(Basilisk* b) : b_{b} {}

  void Run() {
    b_->CommandBoth([](Servo* s) { s->SetQuery(); });

    switch (b_->mux_cr_) {
      case Basilisk::MuxCR::Neokey: {
        NeokeyCommandReceiver::Parse();
        NeokeyCommandReceiver::nk_cmd_ = 0;
      } break;
      case Basilisk::MuxCR::Xbee: {
      } break;
      default:
        break;
    }

    ModeRunners::mode_runners.at(b_->cmd_.mode)(b_);
  }

 private:
  Basilisk* b_;
};
