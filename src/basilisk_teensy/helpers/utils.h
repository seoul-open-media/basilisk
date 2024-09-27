#pragma once

#include <Arduino.h>

#include <map>

#ifndef NaN
#define NaN (0.0 / 0.0)
#endif

using LR = bool;
#define BOOL_L (false)
#define BOOL_R (true)
#define IDX_L (0)
#define IDX_R (1)
#define IDX_LR {0, 1}

#define BOOL_ATTACH (true)
#define BOOL_RELEASE (false)

const uint64_t one_uint64 = static_cast<uint64_t>(1);

void pp(uint8_t& n) {
  if (n < 255) n++;
}

double signedpow(const double& base, const float& exponent) {
  const auto y = pow(abs(base), exponent);
  return base >= 0 ? y : -y;
}

template <typename K, typename V>  // Assumes that V is a pointer type.
V SafeAt(const std::map<K, V>& map, const K& key) {
  auto it = map.find(key);
  if (it == map.end()) {
    return nullptr;
  } else {
    return it->second;
  }
}

double nearest_pmn(const double& tgt, double var) {
  if (isnan(tgt) || isnan(var)) return NaN;
  if (var == tgt) return var;
  if (var > tgt) {
    while (var > tgt + 0.5) var -= 1.0;
  } else {
    while (var < tgt - 0.5) var += 1.0;
  }
  return var;
}

// Time is in milliseconds, stored as uint32_t.
// Continuously usable up to approximately 50 days.
class Beat {
 public:
  Beat(const uint32_t& interval)
      : next_beat_{millis() + interval + random(interval)},
        interval_{interval} {}

  bool Hit() {
    if (millis() >= next_beat_) {
      while (millis() >= next_beat_) next_beat_ += interval_;
      return true;
    } else {
      return false;
    }
  }

 private:
  uint32_t next_beat_;
  const uint32_t interval_;
};

struct Vec2 {
  double x, y;

  Vec2() : x{0.0}, y{0.0} {}

  Vec2(const double& _x, const double& _y) : x{_x}, y{_y} {}

  Vec2(const double& arg) : x{cos(arg)}, y{sin(arg)} {}

  bool isnan() const { return ::isnan(x) || ::isnan(y); }

  double mag() const {
    if (isnan()) return NaN;
    return sqrt(sq(x) + sq(y));
  }

  double arg() const {
    if (isnan()) return NaN;
    return atan2(y, x) / TWO_PI;
  }

  double dist(const Vec2& other) const {
    if (isnan() || other.isnan()) return NaN;
    return sqrt(sq(x - other.x) + sq(y - other.y));
  }

  Vec2 operator+(const Vec2& other) const {
    return Vec2(x + other.x, y + other.y);
  }

  Vec2 operator-(const Vec2& other) const {
    return Vec2(x - other.x, y - other.y);
  }

  Vec2 operator*(const double& scalar) const {
    return Vec2(x * scalar, y * scalar);
  }

  friend Vec2 operator*(const double& scalar, const Vec2& vec) {
    return vec * scalar;
  }

  Vec2 operator/(const double& scalar) const {
    if (scalar == 0.0) return Vec2{NaN, NaN};
    return Vec2(x / scalar, y / scalar);
  }

  void add(const Vec2& other) {
    x += other.x;
    y += other.y;
  }

  void sub(const Vec2& other) {
    x -= other.x;
    y -= other.y;
  }

  void scale(const double& scalar) {
    x *= scalar;
    y *= scalar;
  }

  Vec2 normalize() const {
    const auto m = mag();
    if (m == 0.0 || ::isnan(m)) return Vec2{1.0, 0.0};
    return *this / m;
  }

  double argsub(const Vec2& other) const { return arg() - other.arg(); }

  void print() const {
    Serial.print("Vec2(");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.println(")");
  }
};

template <typename T>
T clamp(const T& val, const T& lb, const T& ub) {
  if (val != val) return NaN;
  if (val < lb) return lb;
  if (val > ub) return ub;
  return val;
}

template <typename T>
class clamped {
 public:
  clamped() { val_ = 0; }

  clamped(const clamped& other) { val_ = other.val_; }

  clamped& operator=(const T& new_val) {
    val_ = clamp(new_val, lb(), ub());
    return *this;
  }

  clamped& operator=(const clamped& other) {
    val_ = clamp(other.val_, lb(), ub());
    return *this;
  }

  clamped& operator=(clamped&& other) {
    val_ = clamp(other.val_, lb(), ub());
    return *this;
  }

  operator T() { return val_; }

  bool isnan() { return val_ != val_; }

 protected:
  T val_;
  virtual T lb() const = 0;
  virtual T ub() const = 0;
};

class Phi : public clamped<double> {
 public:
  Phi(const double& init_val = 0.0) { val_ = clamp(init_val, lb(), ub()); }

  using clamped::operator=;

 private:
  double lb() const final { return -0.3; }
  double ub() const final { return 0.3; }
};

class PhiSpeed : public clamped<double> {
 public:
  PhiSpeed(const double& init_val = 0.0) { val_ = clamp(init_val, lb(), ub()); }

  using clamped::operator=;

 private:
  double lb() const override { return 0.0; }
  double ub() const override { return 0.25; }
};

class PhiAccLim : public clamped<double> {
 public:
  PhiAccLim(const double& init_val = 1.0) {
    val_ = clamp(init_val, lb(), ub());
  }

  using clamped::operator=;

 private:
  double lb() const override { return 0.1; }
  double ub() const override { return 2.0; }
};

class PhiThr : public clamped<double> {
 public:
  PhiThr(const double& init_val = 0.01) { val_ = clamp(init_val, lb(), ub()); }

  using clamped::operator=;

 private:
  double lb() const override { return 0.001; }
  double ub() const override { return 1.0; }
};

class N64 : public clamped<uint8_t> {
 public:
  N64(const uint8_t& init_val = 32) { val_ = clamp(init_val, lb(), ub()); }

  using clamped::operator=;

 private:
  uint8_t lb() const final { return 1; }
  uint8_t ub() const final { return 64; }
};
