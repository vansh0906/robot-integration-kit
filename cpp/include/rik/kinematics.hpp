// rik::kinematics — body-velocity <-> actuator commands for the three partner platform types,
// plus the base-side command watchdog. Header-only, C++17, no dependencies, no allocation, so it
// can run on the partner's embedded motor-control board. Mirrors rik/platforms.py and rik/driver.py;
// tests/test_kinematics.cpp checks it against golden vectors generated from the Python version.
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <string>

namespace rik {

struct Twist { double vx = 0, vy = 0, wz = 0; };
using Act = std::array<double, 4>;   // diff: [wl, wr]; mecanum: [FL, FR, RL, RR]; ackermann: [wheel, steer]

enum class Kind { DiffDrive, Mecanum, Ackermann };

struct Params {
  Kind kind = Kind::DiffDrive;
  double wheel_radius = 0.1;    // m
  double track_width = 0.5;     // m
  double wheel_base = 0.5;      // m
  double max_wheel_speed = 10;  // rad/s
  double max_steer = 0.5;       // rad (Ackermann)
  double max_speed = 1.0;       // m/s
  double max_yaw_rate = 1.0;    // rad/s
};

inline bool holonomic(const Params& p) { return p.kind == Kind::Mecanum; }

// Scale the whole command so speed and yaw rate are within limits; the path curvature is kept.
inline Twist limit_body(const Params& p, Twist c) {
  if (!holonomic(p)) c.vy = 0.0;
  double s = 1.0;
  const double sp = std::hypot(c.vx, c.vy);
  if (sp > p.max_speed) s = std::min(s, p.max_speed / sp);
  if (std::fabs(c.wz) > p.max_yaw_rate) s = std::min(s, p.max_yaw_rate / std::fabs(c.wz));
  return {c.vx * s, c.vy * s, c.wz * s};
}

inline Act to_actuators(const Params& p, const Twist& c, double current_steer = 0.0) {
  const double r = p.wheel_radius;
  switch (p.kind) {
    case Kind::DiffDrive: {
      const double h = p.track_width / 2;
      return {(c.vx - c.wz * h) / r, (c.vx + c.wz * h) / r, 0, 0};
    }
    case Kind::Mecanum: {
      const double k = p.wheel_base / 2 + p.track_width / 2;
      return {(c.vx - c.vy - k * c.wz) / r, (c.vx + c.vy + k * c.wz) / r,
              (c.vx + c.vy - k * c.wz) / r, (c.vx - c.vy + k * c.wz) / r};
    }
    case Kind::Ackermann: {
      const double steer = std::fabs(c.vx) > 1e-3 ? std::atan(p.wheel_base * c.wz / c.vx) : current_steer;
      return {c.vx / r, steer, 0, 0};
    }
  }
  return {};
}

// Wheel-speed saturation: scale all wheels together (keeps curvature). Steering is clamped.
inline Act saturate(const Params& p, Act a) {
  if (p.kind == Kind::Ackermann) {
    a[0] = std::clamp(a[0], -p.max_wheel_speed, p.max_wheel_speed);
    a[1] = std::clamp(a[1], -p.max_steer, p.max_steer);
    return a;
  }
  const int n = p.kind == Kind::Mecanum ? 4 : 2;
  double m = 0;
  for (int i = 0; i < n; ++i) m = std::max(m, std::fabs(a[i]));
  if (m > p.max_wheel_speed)
    for (int i = 0; i < n; ++i) a[i] *= p.max_wheel_speed / m;
  return a;
}

inline Act command(const Params& p, const Twist& c, double current_steer = 0.0) {
  return saturate(p, to_actuators(p, limit_body(p, c), current_steer));
}

// Wheel odometry: actuator state -> body twist.
inline Twist from_actuators(const Params& p, const Act& a) {
  const double r = p.wheel_radius;
  switch (p.kind) {
    case Kind::DiffDrive:
      return {r * (a[0] + a[1]) / 2, 0, r * (a[1] - a[0]) / p.track_width};
    case Kind::Mecanum: {
      const double k = p.wheel_base / 2 + p.track_width / 2;
      return {r / 4 * (a[0] + a[1] + a[2] + a[3]), r / 4 * (-a[0] + a[1] + a[2] - a[3]),
              r / 4 * (-a[0] + a[1] - a[2] + a[3]) / k};
    }
    case Kind::Ackermann: {
      const double v = r * a[0];
      return {v, 0, v * std::tan(a[1]) / p.wheel_base};
    }
  }
  return {};
}

// Base-side safety: command watchdog + latched E-stop (mirrors rik/driver.py BaseDriver).
class Watchdog {
 public:
  explicit Watchdog(double timeout_s = 0.25) : timeout_(timeout_s) {}
  void on_command(double t, const Twist& c) { last_t_ = t; cmd_ = c; have_ = true; }
  void press_estop() { estop_ = latched_ = true; }
  void release_estop() { estop_ = false; }
  bool reset() { if (estop_) return false; latched_ = false; return true; }
  bool tripped(double t) const { return !have_ || t - last_t_ > timeout_; }
  bool latched() const { return latched_; }
  // The command the motors may follow right now (zero when stale or E-stopped).
  Twist output(double t) const { return (latched_ || tripped(t)) ? Twist{} : cmd_; }

 private:
  double timeout_;
  double last_t_ = 0;
  bool have_ = false, estop_ = false, latched_ = false;
  Twist cmd_{};
};

}  // namespace rik
