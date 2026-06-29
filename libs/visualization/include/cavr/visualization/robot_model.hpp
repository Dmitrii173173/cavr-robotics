#pragma once

// Kinematic description of an articulated industrial robot together with the
// forward-kinematics needed to pose the matching glTF asset (see
// assets/robots/<model>/). The data here mirrors the language-agnostic
// <model>.kinematics.json descriptor that ships next to the mesh.

#include <cavr/core/geometry.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <string_view>

namespace cavr::visualization {

inline constexpr double kPi = 3.14159265358979323846;

[[nodiscard]] constexpr double deg_to_rad(double degrees) noexcept {
  return degrees * (kPi / 180.0);
}

struct JointLimit final {
  double lower_rad{};
  double upper_rad{};

  [[nodiscard]] constexpr double clamp(double angle_rad) const noexcept {
    if (angle_rad < lower_rad) return lower_rad;
    if (angle_rad > upper_rad) return upper_rad;
    return angle_rad;
  }
};

// One revolute joint. `origin_m` is the joint frame origin expressed in the
// parent joint frame at the home pose; `axis` is the unit rotation axis in the
// joint-local frame (right-hand rule). At the home pose all joint frames are
// axis-aligned with the world frame.
struct RevoluteJoint final {
  std::string_view name;       // "S", "L", ...
  std::string_view node;       // glTF node to rotate, e.g. "joint_s"
  std::string_view link_node;  // glTF mesh node, e.g. "link_s"
  core::Vec3 origin_m{};
  core::Vec3 axis{};
  JointLimit limit{};
  double max_speed_rad_s{};
};

inline constexpr std::size_t kGp25JointCount = 6;

template <std::size_t N>
struct RobotModel final {
  std::string_view name;
  std::string_view asset_path;  // repository-relative glTF binary
  std::string_view base_node;
  std::string_view tcp_node;
  std::array<RevoluteJoint, N> joints{};
  core::Vec3 tcp_origin_m{};  // tool-centre point relative to the last joint
};

// Yaskawa Motoman GP25 (YRC1000). Frame: Y-up, metres. Axis ranges and speeds
// are from the official GP25 datasheet.
[[nodiscard]] constexpr RobotModel<kGp25JointCount> yaskawa_gp25() noexcept {
  return RobotModel<kGp25JointCount>{
      "Yaskawa Motoman GP25",
      "assets/robots/yaskawa_gp25/gp25.glb",
      "link_base",
      "tcp",
      {{
          {"S", "joint_s", "link_s",
           {0.0, 0.169, 0.0}, {0.0, 1.0, 0.0},
           {deg_to_rad(-180.0), deg_to_rad(180.0)}, deg_to_rad(210.0)},
          {"L", "joint_l", "link_l",
           {-0.157, 0.336, 0.150}, {1.0, 0.0, 0.0},
           {deg_to_rad(-105.0), deg_to_rad(155.0)}, deg_to_rad(210.0)},
          {"U", "joint_u", "link_u",
           {0.157, 0.760, 0.0}, {1.0, 0.0, 0.0},
           {deg_to_rad(-86.0), deg_to_rad(160.0)}, deg_to_rad(265.0)},
          {"R", "joint_r", "link_r",
           {0.0, 0.200, 0.302}, {0.0, 0.0, 1.0},
           {deg_to_rad(-200.0), deg_to_rad(200.0)}, deg_to_rad(420.0)},
          {"B", "joint_b", "link_b",
           {0.0, 0.0, 0.493}, {1.0, 0.0, 0.0},
           {deg_to_rad(-150.0), deg_to_rad(150.0)}, deg_to_rad(420.0)},
          {"T", "joint_t", "link_t",
           {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0},
           {deg_to_rad(-455.0), deg_to_rad(455.0)}, deg_to_rad(885.0)},
      }},
      {0.0, 0.0, 0.101},
  };
}

namespace detail {

// Minimal rigid transform: rotation `r` (row-major 3x3) and translation `t`,
// acting as p' = t + r * p.
struct Rigid final {
  double r[3][3]{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  core::Vec3 t{};
};

[[nodiscard]] inline core::Vec3 apply(const Rigid& m, const core::Vec3& p) noexcept {
  return core::Vec3{
      m.t.x_m + m.r[0][0] * p.x_m + m.r[0][1] * p.y_m + m.r[0][2] * p.z_m,
      m.t.y_m + m.r[1][0] * p.x_m + m.r[1][1] * p.y_m + m.r[1][2] * p.z_m,
      m.t.z_m + m.r[2][0] * p.x_m + m.r[2][1] * p.y_m + m.r[2][2] * p.z_m,
  };
}

[[nodiscard]] inline Rigid mul(const Rigid& a, const Rigid& b) noexcept {
  Rigid out;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      out.r[i][j] = a.r[i][0] * b.r[0][j] + a.r[i][1] * b.r[1][j] + a.r[i][2] * b.r[2][j];
    }
  }
  out.t = apply(a, b.t);
  return out;
}

// Translate(trans) * Rotate(axis, angle): point' = trans + R * point.
[[nodiscard]] inline Rigid joint_transform(const core::Vec3& axis, double angle_rad,
                                           const core::Vec3& trans) noexcept {
  const double n = std::sqrt(axis.x_m * axis.x_m + axis.y_m * axis.y_m + axis.z_m * axis.z_m);
  const double kx = n > 0.0 ? axis.x_m / n : 0.0;
  const double ky = n > 0.0 ? axis.y_m / n : 0.0;
  const double kz = n > 0.0 ? axis.z_m / n : 0.0;
  const double c = std::cos(angle_rad);
  const double s = std::sin(angle_rad);
  const double v = 1.0 - c;

  Rigid m;
  m.r[0][0] = c + kx * kx * v;
  m.r[0][1] = kx * ky * v - kz * s;
  m.r[0][2] = kx * kz * v + ky * s;
  m.r[1][0] = ky * kx * v + kz * s;
  m.r[1][1] = c + ky * ky * v;
  m.r[1][2] = ky * kz * v - kx * s;
  m.r[2][0] = kz * kx * v - ky * s;
  m.r[2][1] = kz * ky * v + kx * s;
  m.r[2][2] = c + kz * kz * v;
  m.t = trans;
  return m;
}

[[nodiscard]] inline core::Pose3D to_pose(const Rigid& m) noexcept {
  const double m00 = m.r[0][0], m01 = m.r[0][1], m02 = m.r[0][2];
  const double m10 = m.r[1][0], m11 = m.r[1][1], m12 = m.r[1][2];
  const double m20 = m.r[2][0], m21 = m.r[2][1], m22 = m.r[2][2];
  const double trace = m00 + m11 + m22;
  double x, y, z, w;
  if (trace > 0.0) {
    const double s = std::sqrt(trace + 1.0) * 2.0;
    w = 0.25 * s;
    x = (m21 - m12) / s;
    y = (m02 - m20) / s;
    z = (m10 - m01) / s;
  } else if (m00 > m11 && m00 > m22) {
    const double s = std::sqrt(1.0 + m00 - m11 - m22) * 2.0;
    w = (m21 - m12) / s;
    x = 0.25 * s;
    y = (m01 + m10) / s;
    z = (m02 + m20) / s;
  } else if (m11 > m22) {
    const double s = std::sqrt(1.0 + m11 - m00 - m22) * 2.0;
    w = (m02 - m20) / s;
    x = (m01 + m10) / s;
    y = 0.25 * s;
    z = (m12 + m21) / s;
  } else {
    const double s = std::sqrt(1.0 + m22 - m00 - m11) * 2.0;
    w = (m10 - m01) / s;
    x = (m02 + m20) / s;
    y = (m12 + m21) / s;
    z = 0.25 * s;
  }
  const double inv = 1.0 / std::sqrt(x * x + y * y + z * z + w * w);
  const auto q = core::Quaternion::from_xyzw(x * inv, y * inv, z * inv, w * inv, 1.0e-6)
                     .value_or(core::Quaternion::identity());
  return core::Pose3D{m.t, q};
}

}  // namespace detail

// World pose of every joint frame plus the tool-centre point for a joint
// configuration `q` (radians, one per joint). The base link is fixed at the
// world origin.
template <std::size_t N>
struct RobotState final {
  std::array<core::Pose3D, N> joints{};
  core::Pose3D tcp{};
};

template <std::size_t N>
[[nodiscard]] inline RobotState<N> forward_kinematics(
    const RobotModel<N>& model, const std::array<double, N>& q) noexcept {
  detail::Rigid accum;  // base frame == world origin
  RobotState<N> state;
  for (std::size_t i = 0; i < N; ++i) {
    const auto& joint = model.joints[i];
    accum = detail::mul(accum, detail::joint_transform(joint.axis, q[i], joint.origin_m));
    state.joints[i] = detail::to_pose(accum);
  }
  detail::Rigid tcp_local;
  tcp_local.t = model.tcp_origin_m;
  state.tcp = detail::to_pose(detail::mul(accum, tcp_local));
  return state;
}

}  // namespace cavr::visualization
