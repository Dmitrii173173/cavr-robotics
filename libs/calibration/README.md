# libs/calibration

The **Calibration-Aware** core of CAVR: the data model that lets a point seen by
the camera be placed in the robot's base frame. Dependency-free maths over
`core::Vec3`/`Pose3D` and the shared `cavr::json`, so it holds without a vendor
SDK. This is the *model*; the estimators that fill it in from a captured
pose+image sequence (e.g. Tsai-Lenz hand-eye) are a later phase.

- **`camera_intrinsics.hpp`** — `CameraIntrinsics` (pinhole `fx,fy,cx,cy` +
  Brown-Conrady radial/tangential distortion) with `project` (camera-frame point →
  pixel), `unproject` (pixel → undistorted ray, iterative), and
  `reprojection_error`. Optical-frame convention: +Z forward, X right, Y down.
- **`hand_eye.hpp`** — `HandEyeCalibration`: the rigid transform plus provenance
  and quality metadata (method, RMS residuals, 1-σ uncertainty, version,
  timestamp). Two mounting styles — `EyeInHand` (camera-in-flange) and `EyeToHand`
  (camera-in-cell) — with `camera_in_base` and `point_base_to_camera` frame
  algebra built on `libs/machine` SE(3) compose/invert.
- **`calibration_io.hpp`** — JSON import/export for both, so a calibration is a
  portable, human-inspectable file that sits next to a machine profile.

Reserved for later phases: intrinsics/hand-eye *estimation* algorithms, live
camera decoding, and wiring vision into the scan → plan step.
