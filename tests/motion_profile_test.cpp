// Scalar velocity profiles: the properties an executor relies on must hold
// analytically over a fine time sweep — position monotonic, speed and
// acceleration bounded by their limits, and the move covers exactly `length` in
// finite time. Covers trapezoidal (with and without a cruise phase, and with
// non-zero boundary speeds) and the jerk-limited S-curve.

#include <cavr/motion/profile.hpp>

#include <cmath>
#include <iostream>
#include <string>
#include <string_view>

namespace {

int failures = 0;

void check(bool condition, std::string_view message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
    ++failures;
  }
}

namespace motion = cavr::motion;

// Sweep a profile and assert the invariants. `v_end` is the expected terminal
// speed (0 for a rest-to-rest move).
void check_invariants(const motion::VelocityProfile& p, double length, double v_max, double a_max,
                      double v_end, std::string_view name) {
  check(p.duration() > 0.0, std::string(name) + ": positive duration");
  check(std::abs(p.length() - length) < 1e-6, std::string(name) + ": covers the full length");
  check(std::abs(p.end_velocity() - v_end) < 1e-6, std::string(name) + ": ends at the boundary speed");
  check(std::abs(p.sample(0.0).s) < 1e-9, std::string(name) + ": starts at s=0");
  check(std::abs(p.sample(p.duration()).s - length) < 1e-6, std::string(name) + ": reaches length");

  const int steps = 2000;
  double prev_s = -1e-9;
  bool monotonic = true, v_ok = true, a_ok = true;
  for (int i = 0; i <= steps; ++i) {
    const double t = p.duration() * i / steps;
    const motion::ProfileSample s = p.sample(t);
    if (s.s < prev_s - 1e-9) monotonic = false;
    prev_s = s.s;
    if (s.v < -1e-6 || s.v > v_max + 1e-4) v_ok = false;
    if (std::abs(s.a) > a_max + 1e-3) a_ok = false;
  }
  check(monotonic, std::string(name) + ": position is monotonic non-decreasing");
  check(v_ok, std::string(name) + ": speed stays within [0, v_max]");
  check(a_ok, std::string(name) + ": |acceleration| stays within a_max");
}

void test_trapezoid_with_cruise() {
  // Long move: v_max is reached, so there is a cruise phase.
  const double L = 10.0, v = 2.0, a = 4.0;
  const auto p = motion::plan_trapezoidal(L, v, a);
  check_invariants(p, L, v, a, 0.0, "trapezoid/cruise");
  // Peak speed hits v_max somewhere in the middle.
  check(std::abs(p.sample(p.duration() * 0.5).v - v) < 1e-3, "trapezoid/cruise: cruises at v_max");
}

void test_trapezoid_triangular() {
  // Short move: v_max is not reached; the profile is triangular (peak < v_max).
  const double L = 0.2, v = 5.0, a = 4.0;
  const auto p = motion::plan_trapezoidal(L, v, a);
  check_invariants(p, L, v, a, 0.0, "trapezoid/triangular");
  double peak = 0.0;
  for (int i = 0; i <= 500; ++i) peak = std::max(peak, p.sample(p.duration() * i / 500).v);
  check(peak < v - 1e-3, "trapezoid/triangular: peak speed stays below v_max");
}

void test_trapezoid_boundary_speeds() {
  // A blended corner: the move does not start or end at rest.
  const double L = 4.0, v = 2.0, a = 4.0, v0 = 1.0, v1 = 1.5;
  const auto p = motion::plan_trapezoidal(L, v, a, v0, v1);
  check_invariants(p, L, v, a, v1, "trapezoid/boundary");
  check(std::abs(p.sample(0.0).v - v0) < 1e-6, "trapezoid/boundary: starts at v0");
}

void test_s_curve() {
  const double L = 10.0, v = 2.0, a = 4.0, j = 8.0;
  const auto p = motion::plan_s_curve(L, v, a, j);
  check_invariants(p, L, v, a, 0.0, "s-curve");

  // Jerk (finite-difference of acceleration) stays within j_max.
  bool jerk_ok = true;
  const double dt = p.duration() / 4000.0;
  for (int i = 1; i < 4000; ++i) {
    const double t = dt * i;
    const double jerk = (p.sample(t + dt).a - p.sample(t - dt).a) / (2 * dt);
    if (std::abs(jerk) > j + 0.5) jerk_ok = false;
  }
  check(jerk_ok, "s-curve: |jerk| stays within j_max");

  // The S-curve is never faster than the trapezoid over the same limits (it adds
  // ramp time), so its duration is at least the trapezoid's.
  const auto trap = motion::plan_trapezoidal(L, v, a);
  check(p.duration() >= trap.duration() - 1e-6, "s-curve: no faster than the trapezoid");
}

void test_degenerate() {
  const auto zero = motion::plan_trapezoidal(0.0, 2.0, 4.0);
  check(zero.empty() && zero.duration() == 0.0, "zero-length move is an empty profile");
  const auto bad = motion::plan_profile(5.0, 0.0, 4.0);
  check(bad.empty(), "non-positive v_max yields an empty profile");
}

}  // namespace

int main() {
  test_trapezoid_with_cruise();
  test_trapezoid_triangular();
  test_trapezoid_boundary_speeds();
  test_s_curve();
  test_degenerate();

  if (failures != 0) {
    std::cerr << failures << " motion profile test(s) failed\n";
    return 1;
  }
  std::cout << "motion profile tests passed\n";
  return 0;
}
