#pragma once

// RobotState (a TelemetryFrame) is one synchronized snapshot of everything the
// controller reports at an instant: joint + Cartesian positions, program/servo
// state, current step, speed, IO, errors, available camera/3D data and any
// events raised. The runtime records a stream of these; the UI renders them.
// This is real telemetry, never synthesized animation.

#include <cavr/core/geometry.hpp>
#include <cavr/core/time.hpp>
#include <cavr/machine/enums.hpp>

#include <string>
#include <vector>

namespace cavr::adapter_sdk {

namespace machine = cavr::machine;

struct IoSample final {
  std::string name;
  double value{0.0};  // 0/1 for digital, scaled value for analog
};

struct ControllerEvent final {
  core::Timestamp timestamp{core::Timestamp::zero()};
  machine::EventKind kind{machine::EventKind::Warning};
  machine::Severity severity{machine::Severity::Info};
  std::string message;
};

struct RobotState final {
  core::Timestamp timestamp{core::Timestamp::zero()};

  std::vector<double> joint_positions;   // rad / m per axis
  core::Pose3D tcp_pose;                  // TCP in base frame

  machine::ProgramState program_state{machine::ProgramState::Idle};
  machine::ServoState servo_state{machine::ServoState::Off};

  int current_step{-1};                   // index into the running task
  std::string current_step_label;
  double speed_fraction{0.0};             // 0..1 of programmed speed
  double tcp_speed_mm_s{0.0};

  std::vector<IoSample> io;

  machine::Severity error_severity{machine::Severity::Info};
  int error_code{0};
  std::string error_message;

  bool has_camera_frame{false};
  std::string camera_frame_id;
  bool has_point_cloud{false};
  std::string point_cloud_id;

  std::vector<ControllerEvent> events;    // events raised on this frame

  [[nodiscard]] bool faulted() const noexcept {
    return error_severity == machine::Severity::Error ||
           error_severity == machine::Severity::Critical;
  }
};

}  // namespace cavr::adapter_sdk
