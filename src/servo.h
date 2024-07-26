#pragma once

#include <ACAN2517FD.h>
#include <Arduino.h>
#include <Moteus.h>

namespace mm = mjbots::moteus;
using PmCmd = mm::PositionMode::Command;
using PmFmt = mm::PositionMode::Format;
using QRpl = mm::Query::Result;
using QFmt = mm::Query::Format;
using Res = mm::Resolution;

// The following pins are selected for the seoul-open-media T4_CanFD board.
#define MCP2518FD_CS 10
#define MCP2518FD_INT 41

// The CAN FD driver (MCP2518FD) object using the ACAN2517FD Arduino library.
ACAN2517FD canfd_driver(MCP2518FD_CS, SPI, MCP2518FD_INT);

class Servo {
 public:
  Servo(const int id, PmFmt* pm_fmt = nullptr, QFmt* q_fmt = nullptr)
      : id_{id},
        controller_{canfd_driver,
                    [=]() {
                      Moteus::Options options;
                      options.id = id;
                      if (q_fmt) {
                        options.query_format = *q_fmt;
                      }
                      return options;
                    }()},
        base_pos_{NaN},
        pm_fmt_{pm_fmt} {}

  void Print() {  // Caution: Does not Query before print.
                  // Querying is left for the Executer.
    const auto& reply = controller_.last_result().values;
    Serial.print(id_);
    Serial.print(F(" : t "));
    Serial.print(millis());
    Serial.print(F(" / mod "));
    Serial.print(static_cast<int>(reply.mode));
    Serial.print(F(" / pos "));
    Serial.print(reply.position);
    Serial.print(F(" / vel "));
    Serial.print(reply.velocity);
    Serial.print(F(" / trq "));
    Serial.print(reply.torque);
    Serial.print(F(" / qcr "));
    Serial.print(reply.q_current);
    Serial.print(F(" / dcr "));
    Serial.print(reply.d_current);
    Serial.print(F(" / apo "));
    Serial.print(reply.abs_position);
    Serial.print(F(" / mtp "));
    Serial.print(reply.motor_temperature);
    Serial.print(F(" / tjc "));
    Serial.print(reply.trajectory_complete);
    Serial.print(F(" / hom "));
    Serial.print(static_cast<int>(reply.home_state));
    Serial.print(F(" / vlt "));
    Serial.print(reply.voltage);
    Serial.print(F(" / tmp "));
    Serial.print(reply.temperature);
    Serial.print(F(" / flt "));
    Serial.print(reply.fault);
    Serial.println();
  }

  bool SetStop() { return controller_.SetStop(); }

  bool SetPosition(PmCmd pm_cmd) {
    return controller_.SetPosition(pm_cmd, pm_fmt_);
  }

  void SetBasePosition() {
    controller_.SetQuery();
    base_pos_ = controller_.last_result().values.position;
  }

  const int id_;
  Moteus controller_;
  double base_pos_;
  PmFmt* pm_fmt_;
};
