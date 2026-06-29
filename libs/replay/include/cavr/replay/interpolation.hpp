#pragma once

#include <cavr/replay/session.hpp>

#include <algorithm>
#include <cmath>
#include <optional>
#include <span>

namespace cavr::replay {

[[nodiscard]] inline core::Vec3 interpolate_position(
    const core::Vec3& from, const core::Vec3& to, double ratio) noexcept {
  return core::Vec3{
      from.x_m + (to.x_m - from.x_m) * ratio,
      from.y_m + (to.y_m - from.y_m) * ratio,
      from.z_m + (to.z_m - from.z_m) * ratio,
  };
}

[[nodiscard]] inline std::optional<core::Quaternion> interpolate_orientation_nlerp(
    const core::Quaternion& from, const core::Quaternion& to, double ratio) noexcept {
  double to_x = to.x();
  double to_y = to.y();
  double to_z = to.z();
  double to_w = to.w();

  const double dot = from.x() * to_x + from.y() * to_y + from.z() * to_z + from.w() * to_w;
  if (dot < 0.0) {
    to_x = -to_x;
    to_y = -to_y;
    to_z = -to_z;
    to_w = -to_w;
  }

  const double x = from.x() + (to_x - from.x()) * ratio;
  const double y = from.y() + (to_y - from.y()) * ratio;
  const double z = from.z() + (to_z - from.z()) * ratio;
  const double w = from.w() + (to_w - from.w()) * ratio;
  const double norm = std::sqrt(x * x + y * y + z * z + w * w);
  if (!std::isfinite(norm) || norm <= 0.0) {
    return std::nullopt;
  }

  return core::Quaternion::from_xyzw(x / norm, y / norm, z / norm, w / norm, 1.0e-9);
}

[[nodiscard]] inline std::optional<core::Pose3D> interpolate_pose(
    const PoseSample& from, const PoseSample& to, core::Timestamp target) noexcept {
  const auto from_ns = from.timestamp.nanoseconds();
  const auto to_ns = to.timestamp.nanoseconds();
  const auto target_ns = target.nanoseconds();

  if (from_ns == to_ns || target_ns < from_ns || target_ns > to_ns) {
    return std::nullopt;
  }

  if (target_ns == from_ns) {
    return from.pose;
  }
  if (target_ns == to_ns) {
    return to.pose;
  }

  const double ratio = static_cast<double>(target_ns - from_ns) / static_cast<double>(to_ns - from_ns);
  auto orientation = interpolate_orientation_nlerp(from.pose.orientation, to.pose.orientation, ratio);
  if (!orientation.has_value()) {
    return std::nullopt;
  }

  return core::Pose3D{
      interpolate_position(from.pose.position_m, to.pose.position_m, ratio),
      *orientation,
  };
}

[[nodiscard]] inline std::optional<core::Pose3D> interpolate_pose_at(
    std::span<const PoseSample> poses, core::Timestamp target) noexcept {
  if (poses.empty()) {
    return std::nullopt;
  }

  if (poses.size() == 1) {
    return poses.front().timestamp == target ? std::optional<core::Pose3D>(poses.front().pose) : std::nullopt;
  }

  const auto it = std::lower_bound(
      poses.begin(), poses.end(), target,
      [](const PoseSample& sample, core::Timestamp timestamp) { return sample.timestamp < timestamp; });

  if (it == poses.begin()) {
    return it->timestamp == target ? std::optional<core::Pose3D>(it->pose) : std::nullopt;
  }
  if (it == poses.end()) {
    return std::nullopt;
  }
  if (it->timestamp == target) {
    return it->pose;
  }

  return interpolate_pose(*(it - 1), *it, target);
}

}  // namespace cavr::replay
