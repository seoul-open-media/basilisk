#pragma once

#include "cmd_rcvrs/neokey_cr.h"
#include "servo_units/basilisk.h"
// #include "exec_funcs/exec_funcs_includes.h"

namespace basilisk {

class Executer {
  using C = Basilisk::Command;
  using M = C::Mode;

 public:
 private:
  C& c = basilisk.cmd_;
  M& m = c.mode;

  void ParseNeokeyCmd() {
    switch (nk_cmd) {
      case 0: {  // None
      } break;
      case 1: {  // Stop
        m = M::Stop;
        c.stop = C::Stop{};
      } break;
      case 2: {  // Em: fix all
        m = M::Em;
        c.em = cmd_presets::em::fix_all;
      } break;
      case 3: {  // Em: free all
        m = M::Em;
        c.em = cmd_presets::em::free_all;
      } break;
      case 4: {  // "d exact -0.25"
        m = M::DExactM025;
        c.d_exact_m025 = C::DExactM025{};
      } break;
      case 5: {  // SetRho: -0.25
        m = M::SetRho;
        c.set_rho = cmd_presets::set_rho::m025;
      } break;
      case 6: {  // SetRho: 0.0
        m = M::SetRho;
        c.set_rho = cmd_presets::set_rho::zero;
      } break;
      case 7: {  // Square walk: 45 degree stride
        m = M::Walk;
        c.walk = cmd_presets::walk::square;
      } break;
      case 8: {  // Cat walk: 90 degree stride
        m = M::Walk;
        c.walk = cmd_presets::walk::catwalk;
      } break;
      case 9: {  // Baby walk: 10 degree stride
        m = M::Walk;
        c.walk = cmd_presets::walk::babywalk;
      } break;
      case 10: {  // Eight walk: feet 45 degree outward
        m = M::Walk;
        c.walk = cmd_presets::walk::eightwalk;
      } break;
      case 11: {  // Diamond: square
        m = M::Diamond;
        c.diamond.fsm_state = C::Diamond::FSMState::Init;
        c.diamond = cmd_presets::diamond::square;
      } break;
      case 12: {  // Gee: right
        m = M::Gee;
        c.gee = cmd_presets::gee::right;
      } break;
      case 13: {  // Gee: left
        m = M::Gee;
        c.gee = cmd_presets::gee::left;
      } break;
      default:
        break;
    }

    nk_cmd = 0;
  }

 public:
  void Run() {
    ParseNeokeyCmd();

    basilisk.CommandBoth([](Servo& s) { s.Query(); });

    switch (m) {
      case M::Stop: {
        ExecFuncs::Stop(basilisk);
      } break;
      case M::Em: {
        ExecFuncs::Em(basilisk);
      } break;
      case M::DExactM025: {
        ExecFuncs::DExactM025(basilisk);
      } break;
      case M::SetRho: {
        ExecFuncs::SetRho(basilisk);
      } break;
      case M::Walk: {
        ExecFuncs::Walk(basilisk);
      } break;
      case M::Diamond: {
        ExecFuncs::Diamond(basilisk);
      } break;
      case M::Gee: {
        ExecFuncs::Gee(basilisk);
      } break;
      default:
        break;
    }
  }
} executer;

}  // namespace basilisk
