#pragma once

// A 3D point cloud captured at an instant — the depth/scan counterpart to
// CameraFrame, so a vision-guided session carries real geometry (a scanned weld
// surface) rather than just a `has_point_cloud` flag. Time-aligned with robot
// telemetry on one clock; the (heavy) point payload belongs in the authoritative
// recording, never in a lightweight metadata catalog.
//
// Points are metres in the named `frame_id` (a logical sensor/robot frame, e.g.
// "weld_cam"); calibration (libs/calibration) is what places them in the robot
// base frame. `colors` and `normals`, when present, are parallel to `points`.

#include <cavr/core/geometry.hpp>
#include <cavr/core/time.hpp>

#include <cstdint>
#include <string>
#include <vector>

namespace cavr::adapter_sdk {

// Per-point RGB, 0-255.
struct PointColor final {
  std::uint8_t r{0};
  std::uint8_t g{0};
  std::uint8_t b{0};
};

struct PointCloud final {
  core::Timestamp timestamp{core::Timestamp::zero()};
  std::string frame_id;                  // logical sensor frame the points live in
  std::vector<core::Vec3> points;        // metres, in `frame_id`
  std::vector<PointColor> colors;        // empty, or one per point
  std::vector<core::Vec3> normals;       // empty, or one per point

  [[nodiscard]] bool empty() const noexcept { return points.empty(); }
  [[nodiscard]] std::size_t size() const noexcept { return points.size(); }
  [[nodiscard]] bool has_colors() const noexcept { return colors.size() == points.size() && !points.empty(); }
  [[nodiscard]] bool has_normals() const noexcept { return normals.size() == points.size() && !points.empty(); }
};

}  // namespace cavr::adapter_sdk
