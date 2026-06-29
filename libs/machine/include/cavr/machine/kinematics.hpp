#pragma once

// Forward kinematics over a profile's axes. Operates directly on AxisSpec so the
// adapter (Cartesian telemetry) and the validator (reachability) share one
// definition that is independent of the rendering layer.

#include <cavr/core/geometry.hpp>
#include <cavr/machine/machine_profile.hpp>

#include <cmath>
#include <cstddef>
#include <vector>

namespace cavr::machine {

namespace detail {

struct Rigid final {
  double r[3][3]{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  core::Vec3 t{};
};

[[nodiscard]] inline core::Vec3 apply(const Rigid& m, const core::Vec3& p) noexcept {
  return core::Vec3{
      m.t.x_m + m.r[0][0] * p.x_m + m.r[0][1] * p.y_m + m.r[0][2] * p.z_m,
      m.t.y_m + m.r[1][0] * p.x_m + m.r[1][1] * p.y_m + m.r[1][2] * p.z_m,
      m.t.z_m + m.r[2][0] * p.x_m + m.r[2][1] * p.y_m + m.r[2][2] * p.z_m};
}

[[nodiscard]] inline Rigid mul(const Rigid& a, const Rigid& b) noexcept {
  Rigid out;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      out.r[i][j] = a.r[i][0] * b.r[0][j] + a.r[i][1] * b.r[1][j] + a.r[i][2] * b.r[2][j];
  out.t = apply(a, b.t);
  return out;
}

[[nodiscard]] inline Rigid revolute(const core::Vec3& axis, double angle, const core::Vec3& trans) noexcept {
  const double n = std::sqrt(axis.x_m * axis.x_m + axis.y_m * axis.y_m + axis.z_m * axis.z_m);
  const double kx = n > 0 ? axis.x_m / n : 0, ky = n > 0 ? axis.y_m / n : 0, kz = n > 0 ? axis.z_m / n : 1;
  const double c = std::cos(angle), s = std::sin(angle), v = 1 - c;
  Rigid m;
  m.r[0][0] = c + kx * kx * v;      m.r[0][1] = kx * ky * v - kz * s; m.r[0][2] = kx * kz * v + ky * s;
  m.r[1][0] = ky * kx * v + kz * s; m.r[1][1] = c + ky * ky * v;      m.r[1][2] = ky * kz * v - kx * s;
  m.r[2][0] = kz * kx * v - ky * s; m.r[2][1] = kz * ky * v + kx * s; m.r[2][2] = c + kz * kz * v;
  m.t = trans;
  return m;
}

[[nodiscard]] inline Rigid prismatic(const core::Vec3& axis, double d, const core::Vec3& trans) noexcept {
  const double n = std::sqrt(axis.x_m * axis.x_m + axis.y_m * axis.y_m + axis.z_m * axis.z_m);
  Rigid m;
  m.t = core::Vec3{trans.x_m + (n > 0 ? axis.x_m / n : 0) * d,
                   trans.y_m + (n > 0 ? axis.y_m / n : 0) * d,
                   trans.z_m + (n > 0 ? axis.z_m / n : 1) * d};
  return m;
}

[[nodiscard]] inline core::Pose3D to_pose(const Rigid& m) noexcept {
  const double tr = m.r[0][0] + m.r[1][1] + m.r[2][2];
  double x, y, z, w;
  if (tr > 0) {
    const double s = std::sqrt(tr + 1.0) * 2;
    w = 0.25 * s; x = (m.r[2][1] - m.r[1][2]) / s; y = (m.r[0][2] - m.r[2][0]) / s; z = (m.r[1][0] - m.r[0][1]) / s;
  } else if (m.r[0][0] > m.r[1][1] && m.r[0][0] > m.r[2][2]) {
    const double s = std::sqrt(1 + m.r[0][0] - m.r[1][1] - m.r[2][2]) * 2;
    w = (m.r[2][1] - m.r[1][2]) / s; x = 0.25 * s; y = (m.r[0][1] + m.r[1][0]) / s; z = (m.r[0][2] + m.r[2][0]) / s;
  } else if (m.r[1][1] > m.r[2][2]) {
    const double s = std::sqrt(1 + m.r[1][1] - m.r[0][0] - m.r[2][2]) * 2;
    w = (m.r[0][2] - m.r[2][0]) / s; x = (m.r[0][1] + m.r[1][0]) / s; y = 0.25 * s; z = (m.r[1][2] + m.r[2][1]) / s;
  } else {
    const double s = std::sqrt(1 + m.r[2][2] - m.r[0][0] - m.r[1][1]) * 2;
    w = (m.r[1][0] - m.r[0][1]) / s; x = (m.r[0][2] + m.r[2][0]) / s; y = (m.r[1][2] + m.r[2][1]) / s; z = 0.25 * s;
  }
  const double inv = 1.0 / std::sqrt(x * x + y * y + z * z + w * w);
  return core::Pose3D{m.t, core::Quaternion::from_xyzw(x * inv, y * inv, z * inv, w * inv, 1.0e-6)
                              .value_or(core::Quaternion::identity())};
}

}  // namespace detail

struct ForwardPose final {
  std::vector<core::Pose3D> joints;  // world pose of each joint frame
  core::Pose3D tcp;                  // tool-centre point
};

// `q` are joint values (rad for revolute, m for prismatic), one per axis.
[[nodiscard]] inline ForwardPose forward_kinematics(
    const std::vector<AxisSpec>& axes, const std::vector<double>& q,
    const core::Vec3& tcp_offset_m = {}) {
  detail::Rigid acc;
  ForwardPose out;
  out.joints.reserve(axes.size());
  for (std::size_t i = 0; i < axes.size(); ++i) {
    const double value = i < q.size() ? q[i] : 0.0;
    const detail::Rigid local = axes[i].type == JointType::Prismatic
                                    ? detail::prismatic(axes[i].axis, value, axes[i].origin_m)
                                    : detail::revolute(axes[i].axis, value, axes[i].origin_m);
    acc = detail::mul(acc, local);
    out.joints.push_back(detail::to_pose(acc));
  }
  detail::Rigid tcp;
  tcp.t = tcp_offset_m;
  out.tcp = detail::to_pose(detail::mul(acc, tcp));
  return out;
}

}  // namespace cavr::machine
