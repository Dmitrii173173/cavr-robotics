// Joint-space trajectory + corner blending. An L-shaped two-segment path (a right
// angle in 2-DoF joint space) is planned with and without a blend on the corner.
// Without a blend the robot stops dead at the corner and passes exactly through
// it; with a blend it keeps its speed up and rounds the corner inside blend_radius.

#include <cavr/motion/trajectory.hpp>

#include <cmath>
#include <iostream>
#include <string_view>
#include <vector>

namespace {

int failures = 0;

void check(bool condition, std::string_view message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
    ++failures;
  }
}

namespace motion = cavr::motion;

double dist(const std::vector<double>& a, const std::vector<double>& b) {
  double s = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i) s += (a[i] - b[i]) * (a[i] - b[i]);
  return std::sqrt(s);
}

// Finite-difference joint speed |dq/dt| at time t.
double speed_at(const motion::Trajectory& tr, double t) {
  const double dt = 1e-4;
  const auto a = tr.sample(t - dt);
  const auto b = tr.sample(t + dt);
  return dist(a, b) / (2 * dt);
}

// An L: (0,0) -> (1,0), then (1,0) -> (1,1). The corner is the knot (1,0).
std::vector<motion::PlanSegment> l_path(double blend_radius) {
  motion::PlanSegment a;
  a.knots = {{0.0, 0.0}, {1.0, 0.0}};
  a.knot_s = {0.0, 1.0};
  a.speed = 1.0;
  a.blend_radius = blend_radius;
  a.command_index = 0;

  motion::PlanSegment b;
  b.knots = {{1.0, 0.0}, {1.0, 1.0}};
  b.knot_s = {0.0, 1.0};
  b.speed = 1.0;
  b.command_index = 1;
  return {a, b};
}

// Scan the middle of the trajectory (past the global start/stop ramps): the
// minimum interior speed, and the closest the path comes to the corner knot.
void scan(const motion::Trajectory& tr, double& min_speed, double& min_corner_dist) {
  const std::vector<double> corner = {1.0, 0.0};
  min_speed = 1e9;
  min_corner_dist = 1e9;
  const double T = tr.duration();
  for (int i = 0; i <= 4000; ++i) {
    const double t = T * i / 4000.0;
    if (t > 0.2 * T && t < 0.8 * T) min_speed = std::min(min_speed, speed_at(tr, t));
    min_corner_dist = std::min(min_corner_dist, dist(tr.sample(t), corner));
  }
}

void test_endpoints() {
  const auto tr = motion::plan(l_path(0.0), motion::MotionLimits::trapezoidal());
  check(!tr.empty() && tr.dof() == 2, "trajectory has two DoF");
  check(dist(tr.sample(-1.0), {0.0, 0.0}) < 1e-9, "before the start holds the first knot");
  check(dist(tr.sample(tr.duration() + 1.0), {1.0, 1.0}) < 1e-9, "after the end holds the last knot");
  check(tr.segment_at(0.0) == 0 && tr.segment_at(tr.duration()) == 1, "segment index advances");
}

void test_blend_vs_stop() {
  const auto limits = motion::MotionLimits::trapezoidal();
  const auto sharp = motion::plan(l_path(0.0), limits);
  const auto blended = motion::plan(l_path(0.3), limits);

  double sharp_speed = 0, sharp_corner = 0, blend_speed = 0, blend_corner = 0;
  scan(sharp, sharp_speed, sharp_corner);
  scan(blended, blend_speed, blend_corner);

  // Sharp corner: the robot comes (almost) to rest at the corner and passes
  // through it exactly.
  check(sharp_speed < 0.1, "without a blend the robot nearly stops at the corner");
  check(sharp_corner < 1e-3, "without a blend the path passes through the corner");

  // Blended corner: speed stays up, the path rounds inside blend_radius, and the
  // whole move takes less time (no stop-and-go).
  check(blend_speed > 0.5, "with a blend the robot keeps its speed through the corner");
  check(blend_corner > 1e-2 && blend_corner < 0.3 + 1e-6,
        "with a blend the path cuts the corner within blend_radius");
  check(blended.duration() < sharp.duration() - 1e-3, "blending shortens the cycle time");
}

}  // namespace

int main() {
  test_endpoints();
  test_blend_vs_stop();

  if (failures != 0) {
    std::cerr << failures << " motion trajectory test(s) failed\n";
    return 1;
  }
  std::cout << "motion trajectory tests passed\n";
  return 0;
}
