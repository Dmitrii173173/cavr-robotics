#pragma once

// Industrial motion vocabulary: a task is an ordered list of MotionCommands the
// controller executes. The application plans and validates these, but the real
// execution is performed by the controller/teach pendant; the app only sends
// the task and monitors telemetry coming back.

#include <cavr/core/geometry.hpp>
#include <cavr/machine/enums.hpp>

#include <optional>
#include <string>
#include <vector>

namespace cavr::machine {

// A motion target can be expressed in joint space or Cartesian space.
struct MotionTarget final {
  std::optional<std::vector<double>> joints;   // rad / m per axis (MoveJ)
  std::optional<core::Pose3D> pose;            // TCP pose (MoveL / MoveC)
};

// Welding parameters attached to a pass (overrides profile defaults).
struct WeldPass final {
  bool enabled{false};
  double travel_speed_mm_s{8.0};
  double segment_length_mm{2.0};
  double tolerance_mm{0.5};
  std::string process_program;
};

struct MotionCommand final {
  MotionKind kind{MotionKind::MoveJ};
  MotionTarget target;             // for MoveJ/MoveL
  std::optional<core::Pose3D> via; // intermediate pose for MoveC
  double speed{0.0};               // rad/s, m/s, or fraction depending on kind
  double blend_radius_m{0.0};
  std::string tool_frame;          // CoordinateFrame name
  std::string user_frame;          // CoordinateFrame name
  double wait_s{0.0};              // for MotionKind::Wait
  std::optional<WeldPass> weld;    // present when this segment is a weld
  std::string label;
};

using MotionTask = std::vector<MotionCommand>;

}  // namespace cavr::machine
