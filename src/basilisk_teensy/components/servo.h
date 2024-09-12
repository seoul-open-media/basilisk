#pragma once

#include "../helpers/imports.h"
#include "canfd_drivers.h"

class Servo : private Moteus {
 public:
  Servo(const int& id, uint8_t bus,  //
        const PmFmt* const pm_fmt, const QFmt* const q_fmt)
      : id_{id},
        Moteus{canfd_drivers[bus - 1],
               [&]() {
                 Options options;
                 options.id = id;
                 options.default_query =
                     false;  // Query right before Command,
                             // not along with Command.
                             // Make sure to pass format_override
                             // argument whenever Querying.
                 return options;
               }()},
        pm_fmt_{pm_fmt},
        q_fmt_{q_fmt} {}

  bool SetQuery() {
    const auto got_rpl = static_cast<Moteus*>(this)->SetQuery(q_fmt_);
    if (got_rpl) SetReply();
    return got_rpl;
  }

  // aux2 position uncoiled.
  QRpl GetReply() {
    auto rpl = last_result().values;
    rpl.abs_position += aux2_revs_;  // Uncoil aux2 position.
    return rpl;
  }

  void SetReply() {
    const auto new_aux2_pos_coiled = last_result().values.abs_position;
    const auto delta_aux2_pos_coiled =
        new_aux2_pos_coiled - prev_aux2_pos_coiled_;
    if (delta_aux2_pos_coiled > 0.5) {
      aux2_revs_--;
    } else if (delta_aux2_pos_coiled < -0.5) {
      aux2_revs_++;
    }
    prev_aux2_pos_coiled_ = new_aux2_pos_coiled;
  }

  void SetPosition(const PmCmd& cmd) {
    static_cast<Moteus*>(this)->SetPosition(cmd, pm_fmt_);
  }

 private:
  const int id_;
  const PmFmt* const pm_fmt_;
  const QFmt* const q_fmt_;
  double prev_aux2_pos_coiled_;  // This field is solely for keeping
                                 // aux2 position of the previous Reply
                                 // in order to compare it to a new Reply
                                 // to track aux2 revolutions, since
                                 // the Moteus Arduino library silently
                                 // updates `last_result_` within `SetQuery()`.
  int aux2_revs_ = 0;

 public:
  void Print() {
    const auto rpl = GetReply();
    Serial.print(id_);
    Serial.print(F(" : t "));
    Serial.print(millis());
    Serial.print(F(" / mod "));
    Serial.print(static_cast<int>(rpl.mode));
    Serial.print(F(" / pos "));
    Serial.print(rpl.position);
    Serial.print(F(" / vel "));
    Serial.print(rpl.velocity);
    Serial.print(F(" / trq "));
    Serial.print(rpl.torque);
    Serial.print(F(" / qcr "));
    Serial.print(rpl.q_current);
    Serial.print(F(" / dcr "));
    Serial.print(rpl.d_current);
    Serial.print(F(" / a2p "));
    Serial.print(rpl.abs_position);
    Serial.print(F(" / mtp "));
    Serial.print(rpl.motor_temperature);
    Serial.print(F(" / tjc "));
    Serial.print(rpl.trajectory_complete);
    Serial.print(F(" / hom "));
    Serial.print(static_cast<int>(rpl.home_state));
    Serial.print(F(" / vlt "));
    Serial.print(rpl.voltage);
    Serial.print(F(" / tmp "));
    Serial.print(rpl.temperature);
    Serial.print(F(" / flt "));
    Serial.print(rpl.fault);
    Serial.print(F(" / a2v "));
    Serial.print(rpl.extra[0].value);
    Serial.print(F(" / evl "));
    Serial.print(rpl.extra[1].value);
    Serial.println();
  }
};
