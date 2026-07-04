// MoveC arc geometry: sampled poses must lie on the circle through the three
// input points, span from start to end passing through the via point, and fall
// back to a straight line when the points are collinear.

#include <cavr/machine/arc.hpp>

#include <cmath>
#include <iostream>
#include <string_view>

namespace {

int failures = 0;

void check(bool condition, std::string_view message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
    ++failures;
  }
}

namespace core = cavr::core;
namespace machine = cavr::machine;

core::Pose3D pose(double x, double y, double z) {
  return core::Pose3D{core::Vec3{x, y, z}, core::Quaternion::identity()};
}

double dist(const core::Vec3& a, const core::Vec3& b) {
  const double dx = a.x_m - b.x_m, dy = a.y_m - b.y_m, dz = a.z_m - b.z_m;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// A half circle in the XY plane: (1,0,0) -> (0,1,0) -> (-1,0,0), centre origin, r=1.
void test_half_circle() {
  const auto path = machine::sample_arc(pose(1, 0, 0), pose(0, 1, 0), pose(-1, 0, 0), 9);
  check(path.is_arc, "half circle is recognised as an arc");
  check(path.poses.size() == 9, "returns the requested sample count");

  bool on_circle = true;
  for (const auto& p : path.poses) {
    if (std::abs(dist(p.position_m, core::Vec3{0, 0, 0}) - 1.0) > 1e-6) on_circle = false;
  }
  check(on_circle, "every sampled point lies on the unit circle");

  // Endpoints match the inputs.
  check(dist(path.poses.front().position_m, core::Vec3{1, 0, 0}) < 1e-9, "starts at the start point");
  check(dist(path.poses.back().position_m, core::Vec3{-1, 0, 0}) < 1e-9, "ends at the end point");

  // Midpoint of a half circle is the via point (0,1,0), a full radius off the
  // straight chord (which runs along the x-axis) — proof it is an arc, not a line.
  const core::Vec3 mid = path.poses[4].position_m;
  check(dist(mid, core::Vec3{0, 1, 0}) < 1e-6, "arc midpoint is the via point");
  check(std::abs(mid.y_m) > 0.9, "arc midpoint is well off the start->end chord");
}

// Collinear points have no circle: the path degrades to a straight line.
void test_collinear_fallback() {
  const auto path = machine::sample_arc(pose(0, 0, 0), pose(1, 0, 0), pose(2, 0, 0), 5);
  check(!path.is_arc, "collinear points fall back to a line");
  check(path.poses.size() == 5, "line still returns the requested samples");
  bool straight = true;
  for (int i = 0; i < 5; ++i) {
    const double expect = 2.0 * i / 4.0;  // 0, 0.5, 1.0, 1.5, 2.0 along x
    if (dist(path.poses[i].position_m, core::Vec3{expect, 0, 0}) > 1e-9) straight = false;
  }
  check(straight, "line samples are evenly spaced along the segment");
}

// An arc in a tilted plane (off the principal axes) stays coplanar with its three
// defining points and hits both endpoints.
void test_tilted_arc() {
  const core::Vec3 a{1, 0, 0}, b{0.707, 0.707, 0.707}, c{0, 0, 1};
  const auto path = machine::sample_arc(pose(a.x_m, a.y_m, a.z_m), pose(b.x_m, b.y_m, b.z_m),
                                        pose(c.x_m, c.y_m, c.z_m), 7);
  check(path.is_arc, "tilted arc is an arc");
  check(dist(path.poses.front().position_m, a) < 1e-9, "tilted arc starts at the start point");
  check(dist(path.poses.back().position_m, c) < 1e-9, "tilted arc ends at the end point");

  // Plane normal of the three points; every sample must lie in that plane.
  const core::Vec3 ab{b.x_m - a.x_m, b.y_m - a.y_m, b.z_m - a.z_m};
  const core::Vec3 ac{c.x_m - a.x_m, c.y_m - a.y_m, c.z_m - a.z_m};
  const core::Vec3 nrm{ab.y_m * ac.z_m - ab.z_m * ac.y_m, ab.z_m * ac.x_m - ab.x_m * ac.z_m,
                       ab.x_m * ac.y_m - ab.y_m * ac.x_m};
  bool coplanar = true;
  for (const auto& p : path.poses) {
    const double d = (p.position_m.x_m - a.x_m) * nrm.x_m + (p.position_m.y_m - a.y_m) * nrm.y_m +
                     (p.position_m.z_m - a.z_m) * nrm.z_m;
    if (std::abs(d) > 1e-6) coplanar = false;
  }
  check(coplanar, "every sample is coplanar with the three defining points");
}

}  // namespace

int main() {
  test_half_circle();
  test_collinear_fallback();
  test_tilted_arc();

  if (failures != 0) {
    std::cerr << failures << " machine arc test(s) failed\n";
    return 1;
  }
  std::cout << "machine arc tests passed\n";
  return 0;
}
