#pragma once

#include <ACAN2517FD.h>
#include <Arduino.h>
#include <Moteus.h>
#include <TeensyThreads.h>

namespace mm = mjbots::moteus;
using PmCmd = mm::PositionMode::Command;
using PmFmt = mm::PositionMode::Format;
using QRpl = mm::Query::Result;
using QFmt = mm::Query::Format;
using Res = mm::Resolution;

// The following pins are selected for the SeoulOpenMedia T4_CanFD board v.1.5.
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

enum class CommandPositionRelativeTo : uint8_t { Absolute, Base, Recent };
enum class ReplyPositionRelativeTo : uint8_t { Absolute, Base };

class Servo : protected Moteus {
 public:
  Servo(int id, uint8_t bus = 1,  //
        PmFmt* pm_fmt = nullptr, PmCmd* pm_cmd_template = nullptr,
        CommandPositionRelativeTo usr_cmd_pos_rel_to =
            CommandPositionRelativeTo::Absolute,
        QFmt* q_fmt = nullptr,
        ReplyPositionRelativeTo usr_rpl_pos_rel_to =
            ReplyPositionRelativeTo::Absolute,
        ReplyPositionRelativeTo usr_rpl_aux2_pos_rel_to =
            ReplyPositionRelativeTo::Absolute)
      : id_{id},
        Moteus{canfd_drivers[bus - 1],
               [=]() {
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
        usr_cmd_pos_rel_to_{usr_cmd_pos_rel_to},
        usr_rpl_pos_rel_to_{usr_rpl_pos_rel_to},
        usr_rpl_aux2_pos_rel_to_{usr_rpl_aux2_pos_rel_to} {}

  QRpl GetReply() { return usr_rpl(/* Mutex lock inside. */); }

  void SetReply() {
    bool new_trjcpt;
    {
      Threads::Scope lock{mutex_};
      const auto delta_aux2_pos =
          last_result().values.abs_position - sys_rpl_.abs_position;
      if (delta_aux2_pos > 0.5) {
        aux2_revs_--;
      } else if (delta_aux2_pos < -0.5) {
        aux2_revs_++;
      }

      new_trjcpt = last_result().values.trajectory_complete;
    }

    update_trjcpt(
        /* Mutex lock inside */ new_trjcpt);

    {
      Threads::Scope lock{mutex_};
      sys_rpl_ = last_result().values;
    }
  }

  bool Query() {
    bool got_rpl;
    {
      Threads::Scope lock{mutex_};
      got_rpl = SetQuery();
    }
    if (got_rpl) SetReply(/* Mutex lock inside. */);
    return got_rpl;
  }

  bool Stop() {
    bool got_rpl;
    {
      Threads::Scope lock{mutex_};
      trjcpt_ = 0;
      got_rpl = SetStop();
    }
    if (got_rpl) {
      SetReply(/* Mutex lock inside. */);
    }
    return got_rpl;
  }

  bool Position(const PmCmd& usr_cmd) {
    bool got_rpl;
    {
      Threads::Scope lock{mutex_};
      trjcpt_ = 0;
      got_rpl = SetPosition(sys_cmd(usr_cmd), pm_fmt_);
    }
    if (got_rpl) SetReply(/* Mutex lock inside. */);
    return got_rpl;
  }

  bool Position(const double& pos) {
    return Position(/* Mutex lock inside. */ sys_cmd([&] {
      auto cmd = pm_cmd_template_ ? *pm_cmd_template_ : PmCmd{};
      cmd.position = pos;
      return cmd;
    }()));
  }

  bool PositionWaitComplete(const PmCmd& usr_cmd) {
    bool complete;
    {
      Threads::Scope lock{mutex_};
      trjcpt_ = 0;
      complete = SetPositionWaitComplete(sys_cmd(usr_cmd), 0.01, pm_fmt_);
    }
    if (complete) SetReply(/* Mutex lock inside. */);
    return complete;
  }

  bool PositionWaitComplete(const double& pos) {
    return PositionWaitComplete(
        /* Mutex lock inside. */ sys_cmd([&] {
          auto cmd = pm_cmd_template_ ? *pm_cmd_template_ : PmCmd{};
          cmd.position = pos;
          return cmd;
        }()));
  }

  bool SetBasePosition() {
    auto got_rpl = Query(/* Mutex lock inside. */);
    if (got_rpl) {
      Threads::Scope lock{mutex_};
      base_pos_ = GetReply().position;
      return true;
    } else {
      return false;
    }
  }

  bool SetBaseAux2Position() {
    auto got_rpl = Query(/* Mutex lock inside. */);
    if (got_rpl) {
      base_aux2_pos_ = GetReply(/* Mutex lock inside. */).abs_position;
      return true;
    } else {
      return false;
    }
  }

  String d(const String& message_in,
           Moteus::DiagnosticReplyMode reply_mode = Moteus::kExpectOK) {
    Threads::Scope lock{mutex_};
    DiagnosticCommand(F("tel stop"));
    SetDiagnosticFlush();
    return DiagnosticCommand(message_in, reply_mode);
  }

  const int id_;

  static Threads::Mutex mutex_;

  PmFmt* pm_fmt_;
  PmCmd* pm_cmd_template_;
  CommandPositionRelativeTo usr_cmd_pos_rel_to_;
  PmCmd sys_cmd(const PmCmd& usr_cmd) {
    auto cmd = usr_cmd;
    switch (usr_cmd_pos_rel_to_) {
      case CommandPositionRelativeTo::Base: {
        if (isnan(base_pos_) || isnan(cmd.position)) break;
        cmd.position += base_pos_;
      } break;
      case CommandPositionRelativeTo::Recent: {
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

  ReplyPositionRelativeTo usr_rpl_pos_rel_to_;
  ReplyPositionRelativeTo usr_rpl_aux2_pos_rel_to_;
  QRpl sys_rpl_;  // Caution: aux2 position coiled.
                  // `SetReply()` should be call after each Query
                  // to track aux2 revolutions. This field is
                  // solely for keeping previos Reply since the
                  // Moteus Arduino library silently updates
                  // `last_result_` within `SetQuery()`.
  QRpl usr_rpl() {
    QRpl rpl;
    {
      Threads::Scope lock{mutex_};
      rpl = sys_rpl_;
    }
    rpl.abs_position += aux2_revs_;  // Uncoil aux2 position.
    if (usr_rpl_pos_rel_to_ == ReplyPositionRelativeTo::Base) {
      rpl.position -= base_pos_;
    }
    if (usr_rpl_aux2_pos_rel_to_ == ReplyPositionRelativeTo::Base) {
      rpl.abs_position -= base_aux2_pos_;
    }
    return rpl;
  }

  double base_pos_;
  double base_aux2_pos_;
  int aux2_revs_ = 0;

  volatile uint8_t trjcpt_ = 0;  // Accumulation of `trajectory_complete`.
  void update_trjcpt(bool increment_or_reset) {
    if (increment_or_reset) {
      Threads::Scope lock{mutex_};
      if (trjcpt_ != 0xFF) {
        trjcpt_++;
      }
    } else {
      Threads::Scope lock{mutex_};
      trjcpt_ = 0;
    }
  }

  void Print() {  // Caution: Does not Query before print.
    const auto rpl = GetReply(/* Mutex lock inside. */);
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
    {
      Threads::Scope lock{mutex_};
      Serial.println(trjcpt_);
    }
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

Threads::Mutex Servo::mutex_;
