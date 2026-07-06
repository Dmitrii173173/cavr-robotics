#pragma once

// Kinematic limits for the planner, expressed as time constants so they are
// unit-agnostic: the same MotionLimits govern a joint sweep (rad) and a Cartesian
// move (mm) without the caller having to convert acceleration between units.
//
//   a_max = v_cruise / accel_time_s      (time to ramp from rest to cruise speed)
//   j_max = a_max     / jerk_time_s       (jerk_time_s > 0 selects the S-curve)
//
// accel_time_s is the visible "smoothness" knob: larger = gentler ramps.

namespace cavr::motion {

struct MotionLimits final {
  double accel_time_s{0.15};  // seconds to reach cruise speed (sets a_max per move)
  double jerk_time_s{0.0};    // > 0 => jerk-limited S-curve accel; 0 => trapezoidal
  double min_hold_s{0.0};     // floor applied to stationary (dwell) segments

  [[nodiscard]] static MotionLimits trapezoidal(double accel_time_s = 0.15) {
    return {accel_time_s, 0.0, 0.0};
  }
  [[nodiscard]] static MotionLimits s_curve(double accel_time_s = 0.15, double jerk_time_s = 0.5) {
    return {accel_time_s, jerk_time_s, 0.0};
  }
};

}  // namespace cavr::motion
