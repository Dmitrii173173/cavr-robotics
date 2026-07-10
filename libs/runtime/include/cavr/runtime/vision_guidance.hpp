#pragma once

// Vision-guided correction: the last link that turns a camera scan into a change
// to the planned weld path — the point of "Vision-Guided" in CAVR. It composes the
// pieces already built: a scan PointCloud (libs/adapter_sdk) captured on a tick,
// the hand-eye calibration + intrinsics model (libs/calibration), and the motion
// task (libs/machine). Steps:
//
//   1. cloud_in_base  — place the scan in the robot base frame using the hand-eye
//      transform and the flange pose at capture time (a camera point is otherwise
//      only meaningful in the sensor frame).
//   2. seam_offset    — compare where the scan actually sees the seam to where the
//      plan expected it, yielding a base-frame correction vector.
//   3. apply_seam_offset — shift the Cartesian targets of the task by that vector,
//      so the robot welds the seam that is really there, not the nominal one.
//
// Dependency-free; the seam estimator here is deliberately simple (a centroid),
// leaving room for a real feature detector later behind the same interface.

#include <cavr/adapter_sdk/point_cloud.hpp>
#include <cavr/calibration/hand_eye.hpp>
#include <cavr/core/geometry.hpp>
#include <cavr/machine/frames.hpp>
#include <cavr/machine/motion.hpp>

#include <string>

namespace cavr::runtime {

namespace sdk = cavr::adapter_sdk;

// Transforms a scan cloud from the camera frame into the robot base frame, using
// the hand-eye calibration and the flange pose at capture time. Colors/normals are
// carried through (normals rotated, not translated); the result is stamped
// "base".
[[nodiscard]] inline sdk::PointCloud cloud_in_base(const sdk::PointCloud& cloud_cam,
                                                   const calibration::HandEyeCalibration& hand_eye,
                                                   const core::Pose3D& flange_in_base) {
  const core::Pose3D cam = calibration::camera_in_base(hand_eye, flange_in_base);

  sdk::PointCloud out;
  out.timestamp = cloud_cam.timestamp;
  out.frame_id = "base";
  out.colors = cloud_cam.colors;
  out.points.reserve(cloud_cam.points.size());
  for (const core::Vec3& p : cloud_cam.points) {
    out.points.push_back(machine::compose(cam, core::Pose3D{p, core::Quaternion::identity()}).position_m);
  }
  if (cloud_cam.has_normals()) {
    out.normals.reserve(cloud_cam.normals.size());
    const core::Pose3D rot_only{core::Vec3{}, cam.orientation};  // rotate directions, don't translate
    for (const core::Vec3& n : cloud_cam.normals) {
      out.normals.push_back(machine::compose(rot_only, core::Pose3D{n, core::Quaternion::identity()}).position_m);
    }
  }
  return out;
}

// Centroid of a cloud (base frame). Zero for an empty cloud.
[[nodiscard]] inline core::Vec3 centroid(const sdk::PointCloud& cloud) {
  if (cloud.points.empty()) return core::Vec3{};
  core::Vec3 sum{};
  for (const core::Vec3& p : cloud.points) {
    sum.x_m += p.x_m;
    sum.y_m += p.y_m;
    sum.z_m += p.z_m;
  }
  const double inv = 1.0 / static_cast<double>(cloud.points.size());
  return core::Vec3{sum.x_m * inv, sum.y_m * inv, sum.z_m * inv};
}

// The base-frame correction from where the plan expected the seam to where the scan
// observed it: observed_seam - expected_seam. `observed_base` is a scan cloud in
// the base frame (from cloud_in_base); the seam is estimated as its centroid.
[[nodiscard]] inline core::Vec3 seam_offset(const sdk::PointCloud& observed_base,
                                            const core::Vec3& expected_seam_base) {
  const core::Vec3 observed = centroid(observed_base);
  return core::Vec3{observed.x_m - expected_seam_base.x_m, observed.y_m - expected_seam_base.y_m,
                    observed.z_m - expected_seam_base.z_m};
}

// Applies a base-frame translation offset to every Cartesian target in a task —
// the vision correction to the planned path. MoveL/MoveC pose targets (and a
// MoveC via point) are shifted; joint-space MoveJ targets and non-motion commands
// are left untouched.
[[nodiscard]] inline machine::MotionTask apply_seam_offset(const machine::MotionTask& task,
                                                           const core::Vec3& offset) {
  auto shift = [&](core::Vec3 p) {
    return core::Vec3{p.x_m + offset.x_m, p.y_m + offset.y_m, p.z_m + offset.z_m};
  };
  machine::MotionTask corrected = task;
  for (machine::MotionCommand& cmd : corrected) {
    if (cmd.target.pose) cmd.target.pose->position_m = shift(cmd.target.pose->position_m);
    if (cmd.via) cmd.via->position_m = shift(cmd.via->position_m);
  }
  return corrected;
}

}  // namespace cavr::runtime
