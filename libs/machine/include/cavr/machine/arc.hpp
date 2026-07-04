#pragma once

// Circular-arc interpolation for MoveC: given three TCP poses (start, an
// intermediate "via", and end), sample poses along the unique circle through the
// three positions. Orientation is interpolated start->end (normalized lerp). When
// the three points are (near-)collinear or coincident the circle is undefined, so
// it falls back to a straight line — a MoveC then behaves like a MoveL.
//
// This is the geometry only: a caller (the controller/mock) solves IK for each
// sampled pose to drive the joints along the path.

#include <cavr/core/geometry.hpp>

#include <cmath>
#include <vector>

namespace cavr::machine {

namespace arc_detail {

using core::Vec3;

[[nodiscard]] inline Vec3 vsub(const Vec3& a, const Vec3& b) {
  return {a.x_m - b.x_m, a.y_m - b.y_m, a.z_m - b.z_m};
}
[[nodiscard]] inline Vec3 vadd(const Vec3& a, const Vec3& b) {
  return {a.x_m + b.x_m, a.y_m + b.y_m, a.z_m + b.z_m};
}
[[nodiscard]] inline Vec3 vmul(const Vec3& a, double s) {
  return {a.x_m * s, a.y_m * s, a.z_m * s};
}
[[nodiscard]] inline double vdot(const Vec3& a, const Vec3& b) {
  return a.x_m * b.x_m + a.y_m * b.y_m + a.z_m * b.z_m;
}
[[nodiscard]] inline Vec3 vcross(const Vec3& a, const Vec3& b) {
  return {a.y_m * b.z_m - a.z_m * b.y_m, a.z_m * b.x_m - a.x_m * b.z_m,
          a.x_m * b.y_m - a.y_m * b.x_m};
}
[[nodiscard]] inline double vnorm(const Vec3& a) { return std::sqrt(vdot(a, a)); }

// Normalized lerp of two orientations, with sign correction so it takes the short
// way. Good enough for a mock/interpolation; not a true slerp (constant speed).
[[nodiscard]] inline core::Quaternion qnlerp(const core::Quaternion& a, const core::Quaternion& b,
                                             double t) {
  const double d = a.x() * b.x() + a.y() * b.y() + a.z() * b.z() + a.w() * b.w();
  const double s = d < 0.0 ? -1.0 : 1.0;
  const double x = a.x() + (s * b.x() - a.x()) * t;
  const double y = a.y() + (s * b.y() - a.y()) * t;
  const double z = a.z() + (s * b.z() - a.z()) * t;
  const double w = a.w() + (s * b.w() - a.w()) * t;
  const double n = std::sqrt(x * x + y * y + z * z + w * w);
  if (n < 1e-12) return a;
  return core::Quaternion::from_xyzw(x / n, y / n, z / n, w / n, 1e-3).value_or(a);
}

}  // namespace arc_detail

struct ArcPath final {
  std::vector<core::Pose3D> poses;  // `count` poses, endpoints included
  bool is_arc{false};               // false when it degenerated to a straight line
};

// Sample `count` (>=2) poses along the arc start -> via -> end.
[[nodiscard]] inline ArcPath sample_arc(const core::Pose3D& start, const core::Pose3D& via,
                                        const core::Pose3D& end, int count) {
  using namespace arc_detail;
  constexpr double kPi = 3.14159265358979323846;
  if (count < 2) count = 2;

  ArcPath out;
  out.poses.reserve(static_cast<std::size_t>(count));

  const Vec3 a = start.position_m;
  const Vec3 b = via.position_m;
  const Vec3 c = end.position_m;
  const Vec3 ab = vsub(b, a);
  const Vec3 ac = vsub(c, a);
  const Vec3 n = vcross(ab, ac);
  const double n2 = vdot(n, n);

  // Collinear / coincident points: no circle — interpolate a straight line a->c.
  if (n2 < 1e-18) {
    for (int i = 0; i < count; ++i) {
      const double t = static_cast<double>(i) / (count - 1);
      core::Pose3D p;
      p.position_m = vadd(a, vmul(vsub(c, a), t));
      p.orientation = qnlerp(start.orientation, end.orientation, t);
      out.poses.push_back(p);
    }
    out.is_arc = false;
    return out;
  }

  // Circumcentre of the triangle a,b,c in 3D, and the circle radius.
  const Vec3 numerator =
      vadd(vmul(vcross(n, ab), vdot(ac, ac)), vmul(vcross(ac, n), vdot(ab, ab)));
  const Vec3 to_center = vmul(numerator, 1.0 / (2.0 * n2));
  const Vec3 center = vadd(a, to_center);
  const double radius = vnorm(to_center);

  // Orthonormal basis in the circle's plane: e1 through `a`, e2 = N x e1.
  const Vec3 e1 = vmul(vsub(a, center), 1.0 / radius);
  const Vec3 unit_n = vmul(n, 1.0 / std::sqrt(n2));
  const Vec3 e2 = vcross(unit_n, e1);

  const auto plane_angle = [&](const Vec3& p) {
    const Vec3 v = vsub(p, center);
    double ang = std::atan2(vdot(v, e2), vdot(v, e1));
    if (ang < 0.0) ang += 2.0 * kPi;
    return ang;  // in [0, 2*pi); a is at angle 0
  };

  const double via_ang = plane_angle(b);
  const double end_ang = plane_angle(c);
  // Sweep from a (angle 0) the way that passes through `via`.
  const double total = (via_ang <= end_ang) ? end_ang : end_ang - 2.0 * kPi;

  for (int i = 0; i < count; ++i) {
    const double t = static_cast<double>(i) / (count - 1);
    const double th = t * total;
    core::Pose3D p;
    p.position_m = vadd(center, vadd(vmul(e1, radius * std::cos(th)), vmul(e2, radius * std::sin(th))));
    p.orientation = qnlerp(start.orientation, end.orientation, t);
    out.poses.push_back(p);
  }
  out.is_arc = true;
  return out;
}

}  // namespace cavr::machine
