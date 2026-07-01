#pragma once

// Numerical inverse kinematics for an arbitrary serial chain, built on the same
// forward_kinematics used everywhere else — so it works for ANY robot profile
// (no per-robot analytic solution), which is what a universal adapter needs.
//
// Method: damped least-squares (Levenberg-Marquardt) over the 6-DoF task error.
// Each iteration builds the 6xN geometric Jacobian by finite-differencing FK,
// solves (J Jᵀ + λ²I) y = e for the 6-vector y, steps q += Jᵀ y, and clamps to
// joint limits. Damping keeps it stable through singularities. Position error is
// in metres, orientation error is the rotation-vector (axis·angle) magnitude in
// radians.

#include <cavr/core/geometry.hpp>
#include <cavr/machine/kinematics.hpp>
#include <cavr/machine/machine_profile.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

namespace cavr::machine {

struct IkResult final {
  std::vector<double> joints;      // solution (seeded and clamped to limits)
  bool converged{false};
  double position_error_m{0.0};
  double orientation_error_rad{0.0};
  int iterations{0};
};

struct IkOptions final {
  int max_iterations{250};
  double position_tolerance_m{1.0e-4};
  double orientation_tolerance_rad{1.0e-3};
  double damping{0.02};            // λ; larger = more stable, slower
  double step_scale{1.0};          // fraction of the DLS step to take each iter
  double fd_epsilon{1.0e-6};       // finite-difference step for the Jacobian
  bool clamp_to_limits{true};
};

namespace detail {

// Rotation matrix from a unit quaternion.
[[nodiscard]] inline std::array<std::array<double, 3>, 3> rot_from_quat(const core::Quaternion& q) {
  const double x = q.x(), y = q.y(), z = q.z(), w = q.w();
  return {{{1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)},
           {2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)},
           {2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)}}};
}

// The rotation-vector (axis·angle) taking rotation `from` to `to`: extracted from
// R = R_to · R_fromᵀ. Magnitude is the angle in radians.
[[nodiscard]] inline core::Vec3 orientation_error(const core::Quaternion& from, const core::Quaternion& to) {
  const auto a = rot_from_quat(to);
  const auto b = rot_from_quat(from);
  // R = a · bᵀ
  std::array<std::array<double, 3>, 3> r{};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      r[i][j] = a[i][0] * b[j][0] + a[i][1] * b[j][1] + a[i][2] * b[j][2];

  const double trace = r[0][0] + r[1][1] + r[2][2];
  const double cos_angle = std::clamp((trace - 1.0) * 0.5, -1.0, 1.0);
  const double angle = std::acos(cos_angle);
  if (angle < 1.0e-9) return core::Vec3{0, 0, 0};

  // axis from the skew-symmetric part
  const double sin_angle = std::sin(angle);
  const double scale = angle / (2.0 * sin_angle);
  return core::Vec3{(r[2][1] - r[1][2]) * scale, (r[0][2] - r[2][0]) * scale, (r[1][0] - r[0][1]) * scale};
}

// Solves the 6x6 system A y = e in place (Gaussian elimination, partial pivot).
// Returns false if singular.
[[nodiscard]] inline bool solve6(std::array<std::array<double, 6>, 6>& a, std::array<double, 6>& e) {
  for (int col = 0; col < 6; ++col) {
    int pivot = col;
    for (int r = col + 1; r < 6; ++r)
      if (std::abs(a[r][col]) > std::abs(a[pivot][col])) pivot = r;
    if (std::abs(a[pivot][col]) < 1.0e-12) return false;
    std::swap(a[col], a[pivot]);
    std::swap(e[col], e[pivot]);
    for (int r = 0; r < 6; ++r) {
      if (r == col) continue;
      const double f = a[r][col] / a[col][col];
      for (int c = col; c < 6; ++c) a[r][c] -= f * a[col][c];
      e[r] -= f * e[col];
    }
  }
  for (int i = 0; i < 6; ++i) e[i] /= a[i][i];
  return true;
}

}  // namespace detail

// Solves for joint values whose TCP pose matches `target`, seeded from `seed`
// (typically the current pose, so the nearest solution is found). `tcp_offset`
// is the tool offset (pose in the flange frame) matching forward_kinematics.
[[nodiscard]] inline IkResult inverse_kinematics(const std::vector<AxisSpec>& axes,
                                                 const core::Pose3D& target,
                                                 const std::vector<double>& seed,
                                                 const core::Pose3D& tcp_offset,
                                                 const IkOptions& options = {}) {
  const std::size_t n = axes.size();
  IkResult result;
  result.joints = seed;
  result.joints.resize(n, 0.0);

  auto tcp_of = [&](const std::vector<double>& q) { return forward_kinematics(axes, q, tcp_offset).tcp; };

  for (int iter = 0; iter < options.max_iterations; ++iter) {
    result.iterations = iter + 1;
    const core::Pose3D current = tcp_of(result.joints);

    // 6D task error: position then orientation (rotation vector).
    std::array<double, 6> e{
        target.position_m.x_m - current.position_m.x_m,
        target.position_m.y_m - current.position_m.y_m,
        target.position_m.z_m - current.position_m.z_m,
        0, 0, 0};
    const core::Vec3 orient = detail::orientation_error(current.orientation, target.orientation);
    e[3] = orient.x_m;
    e[4] = orient.y_m;
    e[5] = orient.z_m;

    result.position_error_m = std::sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
    result.orientation_error_rad = std::sqrt(e[3] * e[3] + e[4] * e[4] + e[5] * e[5]);
    if (result.position_error_m < options.position_tolerance_m &&
        result.orientation_error_rad < options.orientation_tolerance_rad) {
      result.converged = true;
      return result;
    }

    // Geometric Jacobian J (6xN) by finite differences.
    std::vector<std::array<double, 6>> jac(n);
    for (std::size_t j = 0; j < n; ++j) {
      std::vector<double> qp = result.joints;
      qp[j] += options.fd_epsilon;
      const core::Pose3D pp = tcp_of(qp);
      jac[j][0] = (pp.position_m.x_m - current.position_m.x_m) / options.fd_epsilon;
      jac[j][1] = (pp.position_m.y_m - current.position_m.y_m) / options.fd_epsilon;
      jac[j][2] = (pp.position_m.z_m - current.position_m.z_m) / options.fd_epsilon;
      const core::Vec3 dtheta = detail::orientation_error(current.orientation, pp.orientation);
      jac[j][3] = dtheta.x_m / options.fd_epsilon;
      jac[j][4] = dtheta.y_m / options.fd_epsilon;
      jac[j][5] = dtheta.z_m / options.fd_epsilon;
    }

    // A = J Jᵀ + λ²I  (6x6)
    std::array<std::array<double, 6>, 6> a{};
    const double lambda2 = options.damping * options.damping;
    for (int r = 0; r < 6; ++r) {
      for (int c = 0; c < 6; ++c) {
        double sum = 0.0;
        for (std::size_t j = 0; j < n; ++j) sum += jac[j][r] * jac[j][c];
        a[r][c] = sum + (r == c ? lambda2 : 0.0);
      }
    }

    std::array<double, 6> y = e;
    if (!detail::solve6(a, y)) return result;  // singular; give up with best-so-far

    // dq = Jᵀ y, stepped and clamped.
    for (std::size_t j = 0; j < n; ++j) {
      double dq = 0.0;
      for (int r = 0; r < 6; ++r) dq += jac[j][r] * y[r];
      result.joints[j] += options.step_scale * dq;
      if (options.clamp_to_limits) {
        result.joints[j] = std::clamp(result.joints[j], axes[j].lower_limit, axes[j].upper_limit);
      }
    }
  }

  return result;
}

// Convenience overload for a position-only TCP offset.
[[nodiscard]] inline IkResult inverse_kinematics(const std::vector<AxisSpec>& axes,
                                                 const core::Pose3D& target,
                                                 const std::vector<double>& seed,
                                                 const core::Vec3& tcp_offset_m = {},
                                                 const IkOptions& options = {}) {
  return inverse_kinematics(axes, target, seed,
                            core::Pose3D{tcp_offset_m, core::Quaternion::identity()}, options);
}

}  // namespace cavr::machine
