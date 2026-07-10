#pragma once

// Hand-eye calibration — the "where the camera sits relative to the robot" half of
// vision guidance. It is the rigid transform that lets a point seen by the camera
// be expressed in the robot's base frame, so a vision-detected weld seam can be
// turned into robot motion. This header carries the *result* type (the transform
// plus fit-quality / provenance metadata) and the frame algebra to use it; the
// solver that estimates it from a pose+image sequence is a later phase.
//
// Two mounting styles, mirroring the standard hand-eye problem:
//   EyeInHand  — camera bolted to the flange; `transform` is camera-in-flange.
//   EyeToHand  — camera fixed in the cell; `transform` is camera-in-base.

#include <cavr/calibration/camera_intrinsics.hpp>
#include <cavr/core/geometry.hpp>
#include <cavr/machine/frames.hpp>

#include <cstdint>
#include <optional>
#include <string>

namespace cavr::calibration {

enum class HandEyeType { EyeInHand, EyeToHand };

struct HandEyeCalibration final {
  HandEyeType type{HandEyeType::EyeInHand};
  core::Pose3D transform;      // camera-in-flange (EyeInHand) or camera-in-base (EyeToHand)
  std::string camera_frame;    // logical camera name; matches MachineProfile camera

  // Provenance / quality — what makes a calibration "aware" rather than a bare
  // number: how it was obtained and how much to trust it.
  std::string method;                 // e.g. "tsai-lenz", "daniilidis", "manual"
  double residual_position_m{0.0};    // RMS translational fit residual
  double residual_rotation_rad{0.0};  // RMS rotational fit residual
  double uncertainty_position_m{0.0};    // 1-sigma
  double uncertainty_rotation_rad{0.0};  // 1-sigma
  int version{1};
  std::int64_t calibrated_at_ns{0};   // when it was produced (epoch ns)

  [[nodiscard]] bool valid() const noexcept {
    return !camera_frame.empty() && std::isfinite(transform.position_m.x_m) &&
           std::isfinite(transform.position_m.y_m) && std::isfinite(transform.position_m.z_m);
  }
};

// The camera's pose in the base frame, given the current flange pose in base. For
// an eye-to-hand rig the camera is fixed, so the flange pose is irrelevant.
[[nodiscard]] inline core::Pose3D camera_in_base(const HandEyeCalibration& he,
                                                 const core::Pose3D& flange_in_base) {
  switch (he.type) {
    case HandEyeType::EyeInHand:
      return machine::compose(flange_in_base, he.transform);
    case HandEyeType::EyeToHand:
      return he.transform;
  }
  return he.transform;
}

// Express a point given in the base frame in the CAMERA frame, so it can be
// projected through the intrinsics. `flange_in_base` is the robot's current
// flange pose (ignored for eye-to-hand).
[[nodiscard]] inline core::Vec3 point_base_to_camera(const HandEyeCalibration& he,
                                                     const core::Pose3D& flange_in_base,
                                                     const core::Vec3& point_base) {
  const core::Pose3D cam = camera_in_base(he, flange_in_base);
  const core::Pose3D point_pose{point_base, core::Quaternion::identity()};
  // point-in-camera = (camera-in-base)^-1 ∘ point-in-base
  return machine::compose(machine::invert(cam), point_pose).position_m;
}

}  // namespace cavr::calibration
