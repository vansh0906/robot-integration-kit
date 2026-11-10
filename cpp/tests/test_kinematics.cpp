// Tests for rik::kinematics. No framework needed: each CHECK prints and counts failures.
#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "rik/kinematics.hpp"

static int g_fail = 0, g_checks = 0;
#define CHECK(cond)                                                            \
  do {                                                                         \
    ++g_checks;                                                                \
    if (!(cond)) { ++g_fail; std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } \
  } while (0)
#define CHECK_NEAR(a, b, tol) CHECK(std::fabs((a) - (b)) <= (tol))

static std::vector<std::string> split(const std::string& s) {
  std::vector<std::string> out;
  std::stringstream ss(s);
  std::string f;
  while (std::getline(ss, f, ',')) out.push_back(f);
  return out;
}

static std::map<std::string, rik::Params> load_params(const std::string& path) {
  std::map<std::string, rik::Params> m;
  std::ifstream in(path);
  std::string line;
  std::getline(in, line);
  while (std::getline(in, line)) {
    auto c = split(line);
    rik::Params p;
    p.kind = c[1] == "diff_drive" ? rik::Kind::DiffDrive : c[1] == "mecanum" ? rik::Kind::Mecanum : rik::Kind::Ackermann;
    p.wheel_radius = std::stod(c[2]);
    p.track_width = std::stod(c[3]);
    p.wheel_base = std::stod(c[4]);
    p.max_wheel_speed = std::stod(c[5]);
    p.max_steer = std::stod(c[6]) * M_PI / 180.0;
    p.max_speed = std::stod(c[7]);
    p.max_yaw_rate = std::stod(c[8]);
    m[c[0]] = p;
  }
  return m;
}

static void test_golden(const std::string& dir) {
  auto params = load_params(dir + "/platforms.csv");
  CHECK(params.size() == 3);
  std::ifstream in(dir + "/kinematics.csv");
  std::string line;
  std::getline(in, line);
  int rows = 0;
  while (std::getline(in, line)) {
    auto c = split(line);
    const rik::Params& p = params.at(c[1]);
    rik::Twist cmd{std::stod(c[2]), std::stod(c[3]), std::stod(c[4])};
    rik::Act a = rik::command(p, cmd);
    for (int i = 0; i < 4; ++i) CHECK_NEAR(a[i], std::stod(c[5 + i]), 1e-9);
    rik::Twist f = rik::from_actuators(p, a);
    CHECK_NEAR(f.vx, std::stod(c[9]), 1e-9);
    CHECK_NEAR(f.vy, std::stod(c[10]), 1e-9);
    CHECK_NEAR(f.wz, std::stod(c[11]), 1e-9);
    ++rows;
  }
  CHECK(rows == 180);
  std::printf("golden vectors: %d rows checked against the Python implementation\n", rows);
}

static void test_round_trip() {
  rik::Params p;
  p.kind = rik::Kind::Mecanum;
  p.wheel_radius = 0.05; p.track_width = 0.34; p.wheel_base = 0.36; p.max_wheel_speed = 100; p.max_speed = 5; p.max_yaw_rate = 5;
  rik::Twist c{0.3, -0.2, 0.4};
  rik::Twist f = rik::from_actuators(p, rik::command(p, c));
  CHECK_NEAR(f.vx, 0.3, 1e-12);
  CHECK_NEAR(f.vy, -0.2, 1e-12);
  CHECK_NEAR(f.wz, 0.4, 1e-12);
}

static void test_saturation_keeps_curvature() {
  rik::Params p;
  p.kind = rik::Kind::DiffDrive;
  p.wheel_radius = 0.1; p.track_width = 0.5; p.max_wheel_speed = 5; p.max_speed = 10; p.max_yaw_rate = 10;
  rik::Twist c{2.0, 0, 2.0};                       // curvature 1/m, wheels would exceed 5 rad/s
  rik::Act a = rik::command(p, c);
  CHECK(std::fabs(a[0]) <= 5 + 1e-12 && std::fabs(a[1]) <= 5 + 1e-12);
  rik::Twist f = rik::from_actuators(p, a);
  CHECK_NEAR(f.wz / f.vx, 1.0, 1e-12);
}

static void test_ackermann_steer_limit() {
  rik::Params p;
  p.kind = rik::Kind::Ackermann;
  p.wheel_radius = 0.2; p.wheel_base = 1.2; p.max_wheel_speed = 12; p.max_steer = 32 * M_PI / 180; p.max_speed = 2; p.max_yaw_rate = 10;
  rik::Act a = rik::command(p, {1.0, 0, 5.0});     // asks for a 0.2 m radius turn
  CHECK_NEAR(a[1], p.max_steer, 1e-12);
}

static void test_watchdog() {
  rik::Watchdog w(0.25);
  CHECK(w.tripped(0.0));                            // no command yet -> stopped
  w.on_command(1.0, {0.5, 0, 0});
  CHECK_NEAR(w.output(1.2).vx, 0.5, 1e-12);
  CHECK_NEAR(w.output(1.26).vx, 0.0, 1e-12);        // 260 ms without a command -> zero
  w.on_command(2.0, {0.5, 0, 0});
  w.press_estop();
  CHECK_NEAR(w.output(2.01).vx, 0.0, 1e-12);
  CHECK(!w.reset());                                // cannot reset while the button is still pressed
  w.release_estop();
  w.on_command(2.1, {0.5, 0, 0});
  CHECK_NEAR(w.output(2.15).vx, 0.0, 1e-12);        // latched until reset
  CHECK(w.reset());
  CHECK_NEAR(w.output(2.15).vx, 0.5, 1e-12);
}

int main(int argc, char** argv) {
  const std::string golden = argc > 1 ? argv[1] : "../tests/golden";
  test_golden(golden);
  test_round_trip();
  test_saturation_keeps_curvature();
  test_ackermann_steer_limit();
  test_watchdog();
  std::printf("%d checks, %d failed\n", g_checks, g_fail);
  return g_fail ? 1 : 0;
}
