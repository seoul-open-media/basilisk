#pragma once

#include "components/canfd_drivers.h"
#include "helpers/helpers.h"

namespace basilisk {

class Servo : public Moteus {
 public:
  Servo(int id, uint8_t bus, const PmFmt* pm_fmt, const QFmt* q_fmt)
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

  void SetPosition(const PmCmd& cmd) {
    static_cast<Moteus*>(this)->SetPosition(cmd, pm_fmt_);
  }

  QRpl GetReplyAux2PositionUncoiled() {
    auto rpl = last_result().values;
    rpl.abs_position += aux2_revs_;  // Uncoil aux2 position.
    return rpl;
  }

  void SetReply() {
    const auto delta_aux2_pos =
        last_result().values.abs_position - prev_rpl_.abs_position;
    if (delta_aux2_pos > 0.5) {
      aux2_revs_--;
    } else if (delta_aux2_pos < -0.5) {
      aux2_revs_++;
    }

    prev_rpl_ = last_result().values;
  }

  const int id_;
  const PmFmt* const pm_fmt_;
  const QFmt* const q_fmt_;
  QRpl prev_rpl_;  // This field is solely for keeping the previous Reply
                   // in order to compare it to a new Reply to track
                   // aux2 revolutions, since the Moteus Arduino library
                   // silently updates `last_result_` within `SetQuery()`.
  int aux2_revs_ = 0;

 public:
  void Print() {
    const auto rpl = GetReplyAux2PositionUncoiled();
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

}  // namespace basilisk
