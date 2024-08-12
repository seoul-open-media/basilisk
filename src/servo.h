#pragma once

#include <ACAN2517FD.h>
#include <Moteus.h>

namespace mm = mjbots::moteus;
using PmCmd = mm::PositionMode::Command;
using PmFmt = mm::PositionMode::Format;
using QRpl = mm::Query::Result;
using QFmt = mm::Query::Format;
using Res = mm::Resolution;

// The following pins are selected for the SeoulOpenMedia T4_CanFD board v1.5.
// Only Buses 1 and 2 work for now.
#define MCP2518FD_CS_BUS1 10
#define MCP2518FD_INT_BUS1 41
#define MCP2518FD_CS_BUS2 9
#define MCP2518FD_INT_BUS2 42
#define MCP2518FD_CS_BUS3 0
#define MCP2518FD_INT_BUS3 43
#define MCP2518FD_CS_BUS4 2
#define MCP2518FD_INT_BUS4 44

// The CAN FD driver (MCP2518FD) objects using the ACAN2517FD Arduino library.
ACAN2517FD canfd_drivers[4] = {{MCP2518FD_CS_BUS1, SPI, MCP2518FD_INT_BUS1},
                               {MCP2518FD_CS_BUS2, SPI, MCP2518FD_INT_BUS2},
                               {MCP2518FD_CS_BUS3, SPI1, MCP2518FD_INT_BUS3},
                               {MCP2518FD_CS_BUS4, SPI1, MCP2518FD_INT_BUS4}};

enum class PmCmdPosRelTo : uint8_t { Absolute, Base, Recent };
enum class QRplPosRelTo : bool { Absolute, Base };

class Servo : protected Moteus {
 public:
  Servo(int id, uint8_t bus = 1,  //
        PmFmt* pm_fmt = nullptr, PmCmd* pm_cmd_template = nullptr,
        PmCmdPosRelTo usr_pm_cmd_pos_rel_to = PmCmdPosRelTo::Absolute,
        QFmt* q_fmt = nullptr,
        QRplPosRelTo usr_q_rpl_pos_rel_to = QRplPosRelTo::Absolute,
        QRplPosRelTo usr_q_rpl_aux2_pos_rel_to = QRplPosRelTo::Absolute)
      : id_{id},
        Moteus{canfd_drivers[bus - 1],
               [&]() {
                 Moteus::Options options;
                 options.id = id;
                 if (q_fmt) {
                   options.query_format = *q_fmt;
                 }
                 options.default_query = true;
                 return options;
               }()},
        pm_fmt_{pm_fmt},
        pm_cmd_template_{pm_cmd_template},
        usr_pm_cmd_pos_rel_to_{usr_pm_cmd_pos_rel_to},
        usr_q_rpl_pos_rel_to_{usr_q_rpl_pos_rel_to},
        usr_q_rpl_aux2_pos_rel_to_{usr_q_rpl_aux2_pos_rel_to} {}

  QRpl GetQRpl() { return usr_rpl(); }

  void SetQRpl() {
    const auto delta_aux2_pos =
        last_result().values.abs_position - sys_rpl_.abs_position;
    if (delta_aux2_pos > 0.5) {
      aux2_revs_--;
    } else if (delta_aux2_pos < -0.5) {
      aux2_revs_++;
    }

    update_trjcpt(last_result().values.trajectory_complete);

    sys_rpl_ = last_result().values;
  }

  bool Query() {
    const auto got_rpl = SetQuery();
    if (got_rpl) SetQRpl();
    return got_rpl;
  }

  bool Stop() {
    trjcpt_ = 0;
    const auto got_rpl = SetStop();
    if (got_rpl) SetQRpl();
    return got_rpl;
  }

  bool Position(const PmCmd& usr_cmd) {
    trjcpt_ = 0;
    const auto got_rpl = SetPosition(sys_cmd(usr_cmd), pm_fmt_);
    if (got_rpl) SetQRpl();
    return got_rpl;
  }

  bool Position(const double& pos) {
    return Position(sys_cmd([&] {
      auto cmd = pm_cmd_template_ ? *pm_cmd_template_ : PmCmd{};
      cmd.position = pos;
      return cmd;
    }()));
  }

  bool SetBasePosition() {
    const auto got_rpl = Query();
    if (got_rpl) base_pos_ = GetQRpl().position;
    return got_rpl;
  }

  bool SetBaseAux2Position() {
    const auto got_rpl = Query();
    if (got_rpl) base_aux2_pos_ = GetQRpl().abs_position;
    return got_rpl;
  }

  String d(const String& message_in,
           Moteus::DiagnosticReplyMode reply_mode = Moteus::kExpectOK) {
    DiagnosticCommand(F("tel stop"));
    SetDiagnosticFlush();
    return DiagnosticCommand(message_in, reply_mode);
  }

  const int id_;

  PmFmt* pm_fmt_;
  PmCmd* pm_cmd_template_;
  PmCmdPosRelTo usr_pm_cmd_pos_rel_to_;
  PmCmd sys_cmd(const PmCmd& usr_cmd) {
    auto cmd = usr_cmd;
    switch (usr_pm_cmd_pos_rel_to_) {
      case PmCmdPosRelTo::Base: {
        if (isnan(base_pos_) || isnan(cmd.position)) break;
        cmd.position += base_pos_;
      } break;
      case PmCmdPosRelTo::Recent: {
        if (isnan(cmd.position)) break;
        if (micros() - last_result().timestamp < 1e6) {
          cmd.position += last_result().values.position;
        } else {
          Serial.println(F("More that 1s has past since last reply"));
          cmd.position = NaN;
        }
      } break;
      default:
        break;
    }
    return cmd;
  }

  QRplPosRelTo usr_q_rpl_pos_rel_to_;
  QRplPosRelTo usr_q_rpl_aux2_pos_rel_to_;
  QRpl sys_rpl_;  // Caution: aux2 position coiled.
                  // `SetQRpl()` should be call after each Query
                  // to track aux2 revolutions. This field is
                  // solely for keeping previos Reply since the
                  // Moteus Arduino library silently updates
                  // `last_result_` within `SetQuery()`.
  QRpl usr_rpl() {
    auto rpl = sys_rpl_;
    rpl.abs_position += aux2_revs_;  // Uncoil aux2 position.
    if (usr_q_rpl_pos_rel_to_ == QRplPosRelTo::Base) {
      rpl.position -= base_pos_;
    }
    if (usr_q_rpl_aux2_pos_rel_to_ == QRplPosRelTo::Base) {
      rpl.abs_position -= base_aux2_pos_;
    }
    return rpl;
  }

  double base_pos_;
  double base_aux2_pos_;
  int aux2_revs_ = 0;

  volatile uint8_t trjcpt_ = 0;  // Accumulation of `trajectory_complete`.
  void update_trjcpt(bool t_increment_f_reset) {
    if (t_increment_f_reset) {
      if (trjcpt_ != 0xFF) {
        trjcpt_++;
      }
    } else {
      trjcpt_ = 0;
    }
  }

  void Print() {  // Caution: Does not Query before print.
    const auto rpl = GetQRpl();

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
    Serial.print(F(" / apo "));
    Serial.print(rpl.abs_position);
    Serial.print(F(" / mtp "));
    Serial.print(rpl.motor_temperature);
    Serial.print(F(" / tjc "));
    Serial.print(rpl.trajectory_complete);
    Serial.print(F(" +> "));
    Serial.println(trjcpt_);
    Serial.print(F(" / hom "));
    Serial.print(static_cast<int>(rpl.home_state));
    Serial.print(F(" / vlt "));
    Serial.print(rpl.voltage);
    Serial.print(F(" / tmp "));
    Serial.print(rpl.temperature);
    Serial.print(F(" / flt "));
    Serial.print(rpl.fault);
    Serial.println();
  }
};
