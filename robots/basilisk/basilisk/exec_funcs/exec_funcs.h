#pragma once

class Basilisk;

class ExecFuncs {
 public:
  static void Stop(Basilisk&);
  static void Em(Basilisk&);
  static void DExactM025(Basilisk&);
  static void SetRho(Basilisk&);
  static void Walk(Basilisk&);
  static void Diamond(Basilisk&);
  static void Gee(Basilisk&);
};
