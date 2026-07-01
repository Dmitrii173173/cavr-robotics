#pragma once

// Coordinate systems, the controller tool table, and frame-aware Cartesian jog
// math — the model a standard industrial robot exposes: move the TCP by
// (X,Y,Z,Rx,Ry,Rz) expressed in a chosen coordinate system (World, Base, Tool or
// User), with a tool offset (TCP calibration) selected from a fixed set of tool
// slots. This layer computes the target TCP pose in the base frame; inverse
// kinematics (ik.hpp) then turns that into joint values.

#include <cavr/core/geometry.hpp>

#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace cavr::machine {

// The coordinate systems a jog / Cartesian move can be expressed in — the ~4
// standard robot frames plus joint space. Mirrors a vendor controller's CS_*.
enum class CoordinateSystem { Joint, World, Base, Tool, User };

[[nodiscard]] inline const char* to_string(CoordinateSystem cs) {
  switch (cs) {
    case CoordinateSystem::Joint: return "joint";
    case CoordinateSystem::World: return "world";
    case CoordinateSystem::Base: return "base";
    case CoordinateSystem::Tool: return "tool";
    case CoordinateSystem::User: return "user";
  }
  return "base";
}

// One tool slot: the TCP offset (pose of the tool tip in the flange frame) plus
// whether it has been calibrated. A real controller ships these pre-calibrated.
struct ToolSlot final {
  std::string name;
  core::Pose3D tcp_offset;   // tool tip pose relative to the flange
  bool calibrated{false};
};

// The controller's fixed tool table. A standard controller holds a small, fixed
// number of tool registers that can be set (calibrated), cleared and selected.
inline constexpr std::size_t kToolSlotCount = 10;

class ToolTable final {
 public:
  ToolTable() : slots_(kToolSlotCount) {}

  [[nodiscard]] std::size_t size() const noexcept { return slots_.size(); }
  [[nodiscard]] const ToolSlot& slot(std::size_t i) const { return slots_.at(i); }
  [[nodiscard]] int current() const noexcept { return current_; }

  // Calibrate / define a tool slot's TCP offset (marks it calibrated).
  void set_tool(std::size_t i, const core::Pose3D& tcp_offset, std::string name = {}) {
    if (i >= slots_.size()) return;
    slots_[i] = ToolSlot{std::move(name), tcp_offset, true};
  }
  void clear_tool(std::size_t i) {
    if (i < slots_.size()) slots_[i] = ToolSlot{};
  }
  void clear_all() {
    for (auto& s : slots_) s = ToolSlot{};
  }
  [[nodiscard]] bool select(int i) noexcept {
    if (i < 0 || i >= static_cast<int>(slots_.size())) return false;
    current_ = i;
    return true;
  }

  // TCP offset of the currently selected tool (identity if none / uncalibrated).
  [[nodiscard]] core::Pose3D current_offset() const {
    if (current_ < 0 || current_ >= static_cast<int>(slots_.size())) return {};
    return slots_[static_cast<std::size_t>(current_)].tcp_offset;
  }

 private:
  std::vector<ToolSlot> slots_;
  int current_{0};
};

namespace detail {

[[nodiscard]] inline core::Vec3 vadd(const core::Vec3& a, const core::Vec3& b) {
  return {a.x_m + b.x_m, a.y_m + b.y_m, a.z_m + b.z_m};
}

[[nodiscard]] inline core::Quaternion qnorm(double x, double y, double z, double w) {
  const double n = std::sqrt(x * x + y * y + z * z + w * w);
  if (n < 1e-12) return core::Quaternion::identity();
  return core::Quaternion::from_xyzw(x / n, y / n, z / n, w / n, 1e-3).value_or(core::Quaternion::identity());
}

[[nodiscard]] inline core::Quaternion qmul(const core::Quaternion& a, const core::Quaternion& b) {
  return qnorm(a.w() * b.x() + a.x() * b.w() + a.y() * b.z() - a.z() * b.y(),
               a.w() * b.y() - a.x() * b.z() + a.y() * b.w() + a.z() * b.x(),
               a.w() * b.z() + a.x() * b.y() - a.y() * b.x() + a.z() * b.w(),
               a.w() * b.w() - a.x() * b.x() - a.y() * b.y() - a.z() * b.z());
}

[[nodiscard]] inline core::Quaternion qconj(const core::Quaternion& q) {
  return core::Quaternion::from_xyzw(-q.x(), -q.y(), -q.z(), q.w(), 1e-6)
      .value_or(core::Quaternion::identity());
}

// Rotate a vector by a quaternion.
[[nodiscard]] inline core::Vec3 qrotate(const core::Quaternion& q, const core::Vec3& v) {
  const double x = q.x(), y = q.y(), z = q.z(), w = q.w();
  // t = 2 * (q_xyz x v)
  const double tx = 2 * (y * v.z_m - z * v.y_m);
  const double ty = 2 * (z * v.x_m - x * v.z_m);
  const double tz = 2 * (x * v.y_m - y * v.x_m);
  return {v.x_m + w * tx + (y * tz - z * ty), v.y_m + w * ty + (z * tx - x * tz),
          v.z_m + w * tz + (x * ty - y * tx)};
}

// Quaternion from a rotation vector (axis · angle), radians. (Rx,Ry,Rz) jog.
[[nodiscard]] inline core::Quaternion quat_from_rotvec(const core::Vec3& rv) {
  const double angle = std::sqrt(rv.x_m * rv.x_m + rv.y_m * rv.y_m + rv.z_m * rv.z_m);
  if (angle < 1e-12) return core::Quaternion::identity();
  const double s = std::sin(angle * 0.5) / angle;
  return qnorm(rv.x_m * s, rv.y_m * s, rv.z_m * s, std::cos(angle * 0.5));
}

}  // namespace detail

// Rigid pose composition: the pose of b expressed in a's parent (a ∘ b).
[[nodiscard]] inline core::Pose3D compose(const core::Pose3D& a, const core::Pose3D& b) {
  return {detail::vadd(a.position_m, detail::qrotate(a.orientation, b.position_m)),
          detail::qmul(a.orientation, b.orientation)};
}

// Inverse of a rigid pose.
[[nodiscard]] inline core::Pose3D invert(const core::Pose3D& p) {
  const core::Quaternion inv = detail::qconj(p.orientation);
  const core::Vec3 t = detail::qrotate(inv, p.position_m);
  return {{-t.x_m, -t.y_m, -t.z_m}, inv};
}

// Applies a Cartesian jog to the current TCP pose (expressed in the base frame)
// and returns the new target TCP pose in the base frame.
//
//   d_translation  metres along the chosen frame's axes
//   d_rotation     rotation vector (Rx,Ry,Rz), radians, about the chosen frame
//   user_frame_base  the User frame's pose in base (only used for CS::User)
//
// World/Base move in the fixed root frame; Tool moves in the TCP's own frame;
// User moves in the given user frame. (World and Base coincide when the base is
// at the world origin, which is the case here.)
[[nodiscard]] inline core::Pose3D jog_in_frame(const core::Pose3D& current_tcp_base,
                                               CoordinateSystem system, const core::Vec3& d_translation,
                                               const core::Vec3& d_rotation,
                                               const core::Pose3D& user_frame_base = {}) {
  const core::Quaternion dq = detail::quat_from_rotvec(d_rotation);

  switch (system) {
    case CoordinateSystem::Tool: {
      // Motion in the tool's own frame: right-compose the delta.
      const core::Pose3D delta{d_translation, dq};
      return compose(current_tcp_base, delta);
    }
    case CoordinateSystem::User: {
      // Express the TCP in the user frame, jog there, transform back.
      const core::Pose3D cur_in_user = compose(invert(user_frame_base), current_tcp_base);
      const core::Pose3D new_in_user{detail::vadd(cur_in_user.position_m, d_translation),
                                     detail::qmul(dq, cur_in_user.orientation)};
      return compose(user_frame_base, new_in_user);
    }
    case CoordinateSystem::World:
    case CoordinateSystem::Base:
    case CoordinateSystem::Joint:
    default: {
      // Motion in the fixed root frame: translate along base axes, rotate about them.
      return {detail::vadd(current_tcp_base.position_m, d_translation),
              detail::qmul(dq, current_tcp_base.orientation)};
    }
  }
}

}  // namespace cavr::machine
