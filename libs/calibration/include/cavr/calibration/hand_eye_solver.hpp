#pragma once

// Hand-eye estimation (Tsai-Lenz, 1989): recover the rigid camera↔robot transform
// from a set of synchronized (robot pose, camera-of-target pose) samples — the
// algorithm that fills in a HandEyeCalibration from captured data instead of a
// hand-entered number. Solves the classic AX = XB: as the robot moves between
// stations, the gripper motion A and the camera motion B are related by the fixed
// unknown X (camera-in-flange for eye-in-hand).
//
// Dependency-free: the linear algebra (3x3 solves, rotation↔quaternion, skew) is
// hand-rolled here, in the spirit of libs/machine's IK. Method: solve for rotation
// via the modified-Rodrigues form of each motion pair, then translation by least
// squares. Needs at least two motions (three samples) with non-parallel rotation
// axes; more samples reduce noise and are reported in the residual.

#include <cavr/calibration/hand_eye.hpp>
#include <cavr/core/geometry.hpp>
#include <cavr/machine/frames.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace cavr::calibration {

// One capture: the robot's flange pose in base, and the calibration target's pose
// as seen by the camera, recorded on the same instant.
struct HandEyeSample final {
  core::Pose3D flange_in_base;
  core::Pose3D target_in_camera;
};

struct HandEyeSolveResult final {
  HandEyeCalibration calibration;
  bool ok{false};
  int pairs_used{0};
  std::string error;
};

namespace detail {

using Mat3 = std::array<std::array<double, 3>, 3>;

[[nodiscard]] inline Mat3 identity3() {
  return {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
}

[[nodiscard]] inline Mat3 skew(const core::Vec3& v) {
  return {{{0, -v.z_m, v.y_m}, {v.z_m, 0, -v.x_m}, {-v.y_m, v.x_m, 0}}};
}

[[nodiscard]] inline core::Vec3 matvec(const Mat3& m, const core::Vec3& v) {
  return {m[0][0] * v.x_m + m[0][1] * v.y_m + m[0][2] * v.z_m,
          m[1][0] * v.x_m + m[1][1] * v.y_m + m[1][2] * v.z_m,
          m[2][0] * v.x_m + m[2][1] * v.y_m + m[2][2] * v.z_m};
}

[[nodiscard]] inline Mat3 transpose(const Mat3& m) {
  Mat3 t{};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) t[i][j] = m[j][i];
  return t;
}

[[nodiscard]] inline Mat3 matmul(const Mat3& a, const Mat3& b) {
  Mat3 r{};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      for (int k = 0; k < 3; ++k) r[i][j] += a[i][k] * b[k][j];
  return r;
}

// Accumulate S^T S into A and S^T r into b (normal equations for stacked blocks).
inline void accumulate_normal(Mat3& ata, core::Vec3& atb, const Mat3& s, const core::Vec3& r) {
  const Mat3 st = transpose(s);
  const Mat3 sts = matmul(st, s);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) ata[i][j] += sts[i][j];
  const core::Vec3 str = matvec(st, r);
  atb.x_m += str.x_m;
  atb.y_m += str.y_m;
  atb.z_m += str.z_m;
}

// Solves the 3x3 system A x = b (Gaussian elimination, partial pivot). Returns
// false if singular.
[[nodiscard]] inline bool solve3(Mat3 a, core::Vec3 b, core::Vec3& x) {
  std::array<double, 3> rhs{b.x_m, b.y_m, b.z_m};
  for (int col = 0; col < 3; ++col) {
    int pivot = col;
    for (int r = col + 1; r < 3; ++r)
      if (std::abs(a[r][col]) > std::abs(a[pivot][col])) pivot = r;
    if (std::abs(a[pivot][col]) < 1.0e-12) return false;
    std::swap(a[col], a[pivot]);
    std::swap(rhs[static_cast<std::size_t>(col)], rhs[static_cast<std::size_t>(pivot)]);
    for (int r = 0; r < 3; ++r) {
      if (r == col) continue;
      const double f = a[r][col] / a[col][col];
      for (int c = col; c < 3; ++c) a[r][c] -= f * a[col][c];
      rhs[static_cast<std::size_t>(r)] -= f * rhs[static_cast<std::size_t>(col)];
    }
  }
  x = core::Vec3{rhs[0] / a[0][0], rhs[1] / a[1][1], rhs[2] / a[2][2]};
  return true;
}

[[nodiscard]] inline Mat3 quat_to_mat(const core::Quaternion& q) {
  const double x = q.x(), y = q.y(), z = q.z(), w = q.w();
  return {{{1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)},
           {2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)},
           {2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)}}};
}

[[nodiscard]] inline core::Quaternion mat_to_quat(const Mat3& m) {
  const double trace = m[0][0] + m[1][1] + m[2][2];
  double x, y, z, w;
  if (trace > 0.0) {
    double s = std::sqrt(trace + 1.0) * 2.0;
    w = 0.25 * s;
    x = (m[2][1] - m[1][2]) / s;
    y = (m[0][2] - m[2][0]) / s;
    z = (m[1][0] - m[0][1]) / s;
  } else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) {
    double s = std::sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0;
    w = (m[2][1] - m[1][2]) / s;
    x = 0.25 * s;
    y = (m[0][1] + m[1][0]) / s;
    z = (m[0][2] + m[2][0]) / s;
  } else if (m[1][1] > m[2][2]) {
    double s = std::sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0;
    w = (m[0][2] - m[2][0]) / s;
    x = (m[0][1] + m[1][0]) / s;
    y = 0.25 * s;
    z = (m[1][2] + m[2][1]) / s;
  } else {
    double s = std::sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0;
    w = (m[1][0] - m[0][1]) / s;
    x = (m[0][2] + m[2][0]) / s;
    y = (m[1][2] + m[2][1]) / s;
    z = 0.25 * s;
  }
  const double n = std::sqrt(x * x + y * y + z * z + w * w);
  if (n < 1e-12) return core::Quaternion::identity();
  return core::Quaternion::from_xyzw(x / n, y / n, z / n, w / n, 1e-3)
      .value_or(core::Quaternion::identity());
}

// Modified Rodrigues vector Pr = 2 sin(θ/2) · axis of a rotation, read from its
// quaternion (xyz = sin(θ/2)·axis), with the hemisphere fixed so θ ∈ [0, π).
[[nodiscard]] inline core::Vec3 rodrigues(const core::Quaternion& q) {
  const double sign = q.w() < 0.0 ? -1.0 : 1.0;
  return core::Vec3{2 * sign * q.x(), 2 * sign * q.y(), 2 * sign * q.z()};
}

[[nodiscard]] inline double norm(const core::Vec3& v) {
  return std::sqrt(v.x_m * v.x_m + v.y_m * v.y_m + v.z_m * v.z_m);
}

}  // namespace detail

// Solves for the hand-eye transform from the samples. For eye-in-hand the result
// is camera-in-flange; for eye-to-hand the flange poses are inverted first so the
// same AX = XB machinery yields camera-in-base.
[[nodiscard]] inline HandEyeSolveResult solve_hand_eye(const std::vector<HandEyeSample>& samples,
                                                       HandEyeType type = HandEyeType::EyeInHand,
                                                       std::string camera_frame = {}) {
  using namespace detail;
  HandEyeSolveResult result;
  if (samples.size() < 3) {
    result.error = "hand-eye needs at least 3 samples (2 motions)";
    return result;
  }

  // Robot pose used as the "gripper" frame; eye-to-hand inverts it (camera fixed in
  // the cell, so the base moves relative to the flange).
  std::vector<core::Pose3D> hg, hc;
  hg.reserve(samples.size());
  hc.reserve(samples.size());
  for (const auto& s : samples) {
    hg.push_back(type == HandEyeType::EyeToHand ? machine::invert(s.flange_in_base) : s.flange_in_base);
    hc.push_back(s.target_in_camera);
  }

  // --- rotation: skew(Pg+Pc) · Pcg' = Pc - Pg, stacked over consecutive pairs.
  Mat3 rot_ata{};
  core::Vec3 rot_atb{};
  std::vector<core::Pose3D> motion_a, motion_b;
  for (std::size_t i = 0; i + 1 < samples.size(); ++i) {
    const core::Pose3D a = machine::compose(machine::invert(hg[i + 1]), hg[i]);  // gripper motion
    const core::Pose3D b = machine::compose(hc[i + 1], machine::invert(hc[i]));  // camera motion
    motion_a.push_back(a);
    motion_b.push_back(b);
    const core::Vec3 pg = rodrigues(a.orientation);
    const core::Vec3 pc = rodrigues(b.orientation);
    const core::Vec3 sum{pg.x_m + pc.x_m, pg.y_m + pc.y_m, pg.z_m + pc.z_m};
    const core::Vec3 diff{pc.x_m - pg.x_m, pc.y_m - pg.y_m, pc.z_m - pg.z_m};
    accumulate_normal(rot_ata, rot_atb, skew(sum), diff);
  }
  result.pairs_used = static_cast<int>(motion_a.size());

  core::Vec3 pcg_prime{};
  if (!solve3(rot_ata, rot_atb, pcg_prime)) {
    result.error = "degenerate rotations (parallel axes?)";
    return result;
  }
  const double np2 = pcg_prime.x_m * pcg_prime.x_m + pcg_prime.y_m * pcg_prime.y_m +
                     pcg_prime.z_m * pcg_prime.z_m;
  const double scale = 2.0 / std::sqrt(1.0 + np2);
  const core::Vec3 pcg{scale * pcg_prime.x_m, scale * pcg_prime.y_m, scale * pcg_prime.z_m};
  const double n2 = pcg.x_m * pcg.x_m + pcg.y_m * pcg.y_m + pcg.z_m * pcg.z_m;

  // Rcg = (1 - n2/2) I + 0.5 (Pcg Pcgᵀ + sqrt(4 - n2) skew(Pcg))
  Mat3 rcg = identity3();
  const double a_coef = 1.0 - n2 / 2.0;
  const double root = std::sqrt(std::max(0.0, 4.0 - n2));
  const Mat3 sk = skew(pcg);
  const std::array<double, 3> p{pcg.x_m, pcg.y_m, pcg.z_m};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      rcg[i][j] = a_coef * (i == j ? 1.0 : 0.0) + 0.5 * (p[static_cast<std::size_t>(i)] * p[static_cast<std::size_t>(j)] + root * sk[i][j]);

  // --- translation: (Ra - I) Tcg = Rcg·Tc - Tg, stacked least squares.
  Mat3 tr_ata{};
  core::Vec3 tr_atb{};
  for (std::size_t i = 0; i < motion_a.size(); ++i) {
    Mat3 ra = quat_to_mat(motion_a[i].orientation);
    for (int d = 0; d < 3; ++d) ra[d][d] -= 1.0;  // Ra - I
    const core::Vec3 rhs{matvec(rcg, motion_b[i].position_m).x_m - motion_a[i].position_m.x_m,
                         matvec(rcg, motion_b[i].position_m).y_m - motion_a[i].position_m.y_m,
                         matvec(rcg, motion_b[i].position_m).z_m - motion_a[i].position_m.z_m};
    accumulate_normal(tr_ata, tr_atb, ra, rhs);
  }
  core::Vec3 tcg{};
  if (!solve3(tr_ata, tr_atb, tcg)) {
    result.error = "degenerate translations";
    return result;
  }

  HandEyeCalibration& cal = result.calibration;
  cal.type = type;
  cal.camera_frame = std::move(camera_frame);
  cal.transform = core::Pose3D{tcg, mat_to_quat(rcg)};
  cal.method = "tsai-lenz";

  // Residual: how well A·X = X·B holds across the pairs (RMS position / rotation).
  double pos_sq = 0.0, rot_sq = 0.0;
  for (std::size_t i = 0; i < motion_a.size(); ++i) {
    const core::Pose3D ax = machine::compose(motion_a[i], cal.transform);
    const core::Pose3D xb = machine::compose(cal.transform, motion_b[i]);
    const core::Vec3 dp{ax.position_m.x_m - xb.position_m.x_m, ax.position_m.y_m - xb.position_m.y_m,
                        ax.position_m.z_m - xb.position_m.z_m};
    pos_sq += dp.x_m * dp.x_m + dp.y_m * dp.y_m + dp.z_m * dp.z_m;
    // rotation error magnitude between the two orientations
    const core::Quaternion rel =
        machine::compose(machine::invert(ax), xb).orientation;
    const double ang = 2.0 * std::acos(std::min(1.0, std::abs(rel.w())));
    rot_sq += ang * ang;
  }
  const double inv_n = 1.0 / static_cast<double>(motion_a.size());
  cal.residual_position_m = std::sqrt(pos_sq * inv_n);
  cal.residual_rotation_rad = std::sqrt(rot_sq * inv_n);

  result.ok = true;
  return result;
}

}  // namespace cavr::calibration
