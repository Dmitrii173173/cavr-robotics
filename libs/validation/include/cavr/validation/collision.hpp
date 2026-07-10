#pragma once

// Collision geometry for pre-execution validation. The robot is approximated as a
// chain of capsules — one per link (a segment between consecutive joint origins,
// swept by a radius) plus the tool stub — and checked, at a single joint
// configuration, against itself and the cell. This is the honest first collision
// model the validator advertises: exact segment/segment and segment/sphere/plane
// distances (no meshes), so it never silently passes a self-intersection or a
// crash into the floor or a known obstacle.
//
// Primitives are chosen so every test is exact and cheap:
//   - self-collision: non-adjacent link capsules vs each other,
//   - floor: a world z-plane with optional clearance,
//   - obstacles: spheres (a fixture bounding sphere, a post, a nozzle).
// Boxes/meshes are intentionally not modelled yet rather than approximated.

#include <cavr/core/geometry.hpp>
#include <cavr/machine/kinematics.hpp>
#include <cavr/machine/machine_profile.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace cavr::validation {

namespace machine = cavr::machine;

struct SphereObstacle final {
  core::Vec3 center;
  double radius_m{0.0};
  std::string name{"obstacle"};
};

struct CollisionModel final {
  double link_radius_m{0.06};   // capsule radius approximating each robot link
  bool check_self{true};
  bool check_floor{true};
  // The floor is a plane perpendicular to the "up" axis at `floor_level_m`; links
  // must stay above it by `floor_clearance_m`. up_axis 0/1/2 = x/y/z (the GP25
  // asset is Y-up, so its floor is the y-plane).
  int floor_up_axis{2};
  double floor_level_m{0.0};
  double floor_clearance_m{0.0};
  std::vector<SphereObstacle> spheres;
};

// One detected interference: the two things that touch, how deep (metres, positive
// = penetration past the allowed clearance), and roughly where.
struct CollisionHit final {
  std::string a;
  std::string b;
  double penetration_m{0.0};
  core::Vec3 where;
};

namespace detail {

[[nodiscard]] inline core::Vec3 vsub(const core::Vec3& a, const core::Vec3& b) {
  return {a.x_m - b.x_m, a.y_m - b.y_m, a.z_m - b.z_m};
}
[[nodiscard]] inline core::Vec3 vadd(const core::Vec3& a, const core::Vec3& b) {
  return {a.x_m + b.x_m, a.y_m + b.y_m, a.z_m + b.z_m};
}
[[nodiscard]] inline core::Vec3 vscale(const core::Vec3& a, double s) {
  return {a.x_m * s, a.y_m * s, a.z_m * s};
}
[[nodiscard]] inline double vdot(const core::Vec3& a, const core::Vec3& b) {
  return a.x_m * b.x_m + a.y_m * b.y_m + a.z_m * b.z_m;
}
[[nodiscard]] inline double clamp01(double x) { return x < 0.0 ? 0.0 : (x > 1.0 ? 1.0 : x); }

// A robot link modelled as a capsule: the segment [p0, p1] swept by `radius`.
struct Capsule final {
  core::Vec3 p0;
  core::Vec3 p1;
  double radius{0.0};
  std::string name;
};

// Squared distance between segments [p1,q1] and [p2,q2], with the closest points.
// Ericson, Real-Time Collision Detection.
[[nodiscard]] inline double closest_segments(const core::Vec3& p1, const core::Vec3& q1,
                                             const core::Vec3& p2, const core::Vec3& q2,
                                             core::Vec3& c1, core::Vec3& c2) {
  const core::Vec3 d1 = vsub(q1, p1), d2 = vsub(q2, p2), r = vsub(p1, p2);
  const double a = vdot(d1, d1), e = vdot(d2, d2), f = vdot(d2, r);
  const double eps = 1e-12;
  double s = 0.0, t = 0.0;
  if (a <= eps && e <= eps) {
    c1 = p1;
    c2 = p2;
    return vdot(vsub(c1, c2), vsub(c1, c2));
  }
  if (a <= eps) {
    t = clamp01(f / e);
  } else {
    const double c = vdot(d1, r);
    if (e <= eps) {
      s = clamp01(-c / a);
    } else {
      const double b = vdot(d1, d2);
      const double denom = a * e - b * b;
      s = denom > eps ? clamp01((b * f - c * e) / denom) : 0.0;
      t = (b * s + f) / e;
      if (t < 0.0) {
        t = 0.0;
        s = clamp01(-c / a);
      } else if (t > 1.0) {
        t = 1.0;
        s = clamp01((b - c) / a);
      }
    }
  }
  c1 = vadd(p1, vscale(d1, s));
  c2 = vadd(p2, vscale(d2, t));
  return vdot(vsub(c1, c2), vsub(c1, c2));
}

// Distance from point p to segment [a,b], with the closest point on the segment.
[[nodiscard]] inline double closest_point_segment(const core::Vec3& p, const core::Vec3& a,
                                                  const core::Vec3& b, core::Vec3& closest) {
  const core::Vec3 ab = vsub(b, a);
  const double denom = vdot(ab, ab);
  const double t = denom > 1e-12 ? clamp01(vdot(vsub(p, a), ab) / denom) : 0.0;
  closest = vadd(a, vscale(ab, t));
  return std::sqrt(vdot(vsub(p, closest), vsub(p, closest)));
}

// The robot's link capsules for one configuration: base→joint0, joint(i-1)→joint(i),
// and the last joint→TCP tool stub.
[[nodiscard]] inline std::vector<Capsule> link_capsules(const std::vector<machine::AxisSpec>& axes,
                                                        const std::vector<double>& q,
                                                        const core::Pose3D& tool, double radius) {
  const machine::ForwardPose fk = machine::forward_kinematics(axes, q, tool);
  std::vector<core::Vec3> points;
  points.reserve(fk.joints.size() + 2);
  points.push_back(core::Vec3{});  // base origin
  for (const auto& j : fk.joints) points.push_back(j.position_m);
  points.push_back(fk.tcp.position_m);

  std::vector<Capsule> caps;
  for (std::size_t k = 0; k + 1 < points.size(); ++k) {
    std::string name;
    if (k == 0) {
      name = "base";
    } else if (k <= axes.size()) {
      name = "link " + axes[k - 1].name;
    } else {
      name = "tool";
    }
    caps.push_back({points[k], points[k + 1], radius, std::move(name)});
  }
  return caps;
}

}  // namespace detail

// Checks a single joint configuration and returns every interference found.
[[nodiscard]] inline std::vector<CollisionHit> check_configuration(
    const std::vector<machine::AxisSpec>& axes, const std::vector<double>& q, const core::Pose3D& tool,
    const CollisionModel& model) {
  using namespace detail;
  std::vector<CollisionHit> hits;
  const std::vector<Capsule> caps = link_capsules(axes, q, tool, model.link_radius_m);

  // Self-collision: non-adjacent capsules (adjacent links share a joint).
  if (model.check_self) {
    for (std::size_t i = 0; i < caps.size(); ++i) {
      for (std::size_t j = i + 2; j < caps.size(); ++j) {
        core::Vec3 c1, c2;
        const double dist = std::sqrt(closest_segments(caps[i].p0, caps[i].p1, caps[j].p0, caps[j].p1, c1, c2));
        const double min_dist = caps[i].radius + caps[j].radius;
        if (dist < min_dist) {
          hits.push_back({caps[i].name, caps[j].name, min_dist - dist,
                          vscale(vadd(c1, c2), 0.5)});
        }
      }
    }
  }

  // Floor: any link dipping below the plane + clearance, along the up axis.
  if (model.check_floor) {
    auto up = [&](const core::Vec3& v) {
      return model.floor_up_axis == 0 ? v.x_m : (model.floor_up_axis == 1 ? v.y_m : v.z_m);
    };
    const double limit = model.floor_level_m + model.floor_clearance_m;
    for (const Capsule& c : caps) {
      const double lowest = std::min(up(c.p0), up(c.p1)) - c.radius;
      if (lowest < limit) {
        const core::Vec3 where = up(c.p0) < up(c.p1) ? c.p0 : c.p1;
        hits.push_back({c.name, "floor", limit - lowest, where});
      }
    }
  }

  // Sphere obstacles.
  for (const SphereObstacle& s : model.spheres) {
    for (const Capsule& c : caps) {
      core::Vec3 closest;
      const double dist = closest_point_segment(s.center, c.p0, c.p1, closest);
      const double min_dist = c.radius + s.radius_m;
      if (dist < min_dist) hits.push_back({c.name, s.name, min_dist - dist, closest});
    }
  }

  return hits;
}

}  // namespace cavr::validation
