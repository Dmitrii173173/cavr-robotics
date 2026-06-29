#pragma once

// The timeline is the production workflow: an ordered list of OperationSteps
// (scan, calibration, point-cloud capture, trajectory planning, validation,
// robot motion, welding, IO, pauses, ...) plus the stream of TimelineEvents
// recorded while the controller executes them. The app plans the steps; the
// controller performs the motion; events flow back from telemetry.

#include <cavr/adapter_sdk/robot_state.hpp>
#include <cavr/core/time.hpp>
#include <cavr/machine/enums.hpp>
#include <cavr/machine/motion.hpp>

#include <array>
#include <string>
#include <string_view>
#include <vector>

namespace cavr::runtime {

namespace machine = cavr::machine;
namespace sdk = cavr::adapter_sdk;

enum class OperationKind {
  Scan, Calibration, PointCloudCapture, TrajectoryPlanning, Validation,
  RobotMotion, Welding, IoEvent, Pause, Warning, Error, Replay
};

namespace detail {
inline constexpr std::array<std::pair<OperationKind, std::string_view>, 12> kOperationKind{{
    {OperationKind::Scan, "scan"}, {OperationKind::Calibration, "calibration"},
    {OperationKind::PointCloudCapture, "point_cloud_capture"},
    {OperationKind::TrajectoryPlanning, "trajectory_planning"},
    {OperationKind::Validation, "validation"}, {OperationKind::RobotMotion, "robot_motion"},
    {OperationKind::Welding, "welding"}, {OperationKind::IoEvent, "io_event"},
    {OperationKind::Pause, "pause"}, {OperationKind::Warning, "warning"},
    {OperationKind::Error, "error"}, {OperationKind::Replay, "replay"}}};
}  // namespace detail

[[nodiscard]] inline std::string to_string(OperationKind v) {
  return machine::detail::enum_to_string(v, detail::kOperationKind);
}
[[nodiscard]] inline OperationKind operation_kind_from_string(std::string_view s) {
  return machine::detail::enum_from_string(s, detail::kOperationKind, OperationKind::RobotMotion);
}

// A planned step on the timeline. Motion steps carry the MotionCommands that
// get handed to the controller; analysis steps (scan, validate, ...) carry none.
struct OperationStep final {
  int id{0};
  OperationKind kind{OperationKind::RobotMotion};
  std::string label;
  double planned_duration_s{0.0};
  machine::MotionTask motion;        // empty for non-motion steps
  std::string notes;
};

// An event that occurred during execution/replay, tied to a step when known.
struct TimelineEvent final {
  core::Timestamp timestamp{core::Timestamp::zero()};
  machine::EventKind kind{machine::EventKind::Warning};
  machine::Severity severity{machine::Severity::Info};
  std::string message;
  int step_index{-1};
};

struct Timeline final {
  std::vector<OperationStep> steps;
  std::vector<TimelineEvent> events;

  // Flatten every motion step's commands into one task for the controller.
  [[nodiscard]] machine::MotionTask to_motion_task() const {
    machine::MotionTask task;
    for (const auto& step : steps) {
      for (const auto& cmd : step.motion) task.push_back(cmd);
    }
    return task;
  }

  void add_controller_event(const sdk::ControllerEvent& e, int step_index) {
    events.push_back({e.timestamp, e.kind, e.severity, e.message, step_index});
  }

  [[nodiscard]] double planned_duration_s() const noexcept {
    double total = 0.0;
    for (const auto& s : steps) total += s.planned_duration_s;
    return total;
  }
};

}  // namespace cavr::runtime
