#pragma once

#include "../helpers/imports.h"
#include "canfd_drivers.h"

class Servo : public Moteus {
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

  bool SetQuery() { return static_cast<Moteus*>(this)->SetQuery(q_fmt_); }

  // aux2 position uncoiled.
  QRpl GetReply() {
    auto rpl = last_result().values;
    if (rpl.abs_position > 0.25) rpl.abs_position -= 1.0;
    return rpl;
  }

  void SetPosition(const PmCmd& cmd) {
    static_cast<Moteus*>(this)->SetPosition(cmd, pm_fmt_);
  }

 private:
  const int id_;
  const PmFmt* const pm_fmt_;
  const QFmt* const q_fmt_;

 public:
  void Print() {
    const auto rpl = GetReply();
    Serial.print(id_);
    Serial.print(F(" : t "));
    Serial.print(millis());
    Serial.print(F(" / mod "));
    Serial.print(static_cast<int>(rpl.mode));
    Serial.print(F(" / pos "));
    Serial.print(rpl.position, 3);
    Serial.print(F(" / vel "));
    Serial.print(rpl.velocity, 3);
    Serial.print(F(" / trq "));
    Serial.print(rpl.torque, 3);
    Serial.print(F(" / qcr "));
    Serial.print(rpl.q_current);
    Serial.print(F(" / dcr "));
    Serial.print(rpl.d_current);
    Serial.print(F(" / a2p "));
    Serial.print(rpl.abs_position, 3);
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
    Serial.print(rpl.extra[0].value, 3);
    Serial.print(F(" / evl "));
    Serial.print(static_cast<uint8_t>(rpl.extra[1].value), BIN);
    Serial.println();
  }
};
