#pragma once

// MachineProfile is the persistent configuration for one physical robot/machine.
// It is produced by connecting to a controller and discovering its capabilities
// (or authored by hand), then saved so the same machine can be reconnected and
// reused without manual setup. It is intentionally controller-neutral: adapters
// map vendor-specific variables onto these structures.

#include <cavr/core/geometry.hpp>
#include <cavr/machine/enums.hpp>

#include <string>
#include <vector>

namespace cavr::machine {

// One controllable axis / joint of the kinematic chain.
struct AxisSpec final {
  std::string name;                 // "S", "L", ... or "J1"
  JointType type{JointType::Revolute};
  core::Vec3 axis{0.0, 0.0, 1.0};   // unit axis in the joint-local frame
  core::Vec3 origin_m{};            // origin relative to the parent joint (home pose)
  double lower_limit{-3.14159265358979};   // rad (revolute) or m (prismatic)
  double upper_limit{3.14159265358979};
  double max_speed{3.14159265358979};       // rad/s or m/s
  std::string controller_variable;  // vendor channel this axis maps to
};

// A named coordinate frame (base, tool, user, camera, object, ...).
struct CoordinateFrame final {
  std::string name;
  FrameKind kind{FrameKind::User};
  std::string parent;               // name of the parent frame ("" for world)
  core::Pose3D transform;           // pose of this frame in the parent frame
};

// Digital/analog IO point mapped from a controller signal.
struct IoChannel final {
  std::string name;
  IoKind kind{IoKind::Digital};
  IoDirection direction{IoDirection::Input};
  int index{0};
  std::string controller_variable;
};

// A telemetry stream the controller exposes and the app subscribes to.
struct TelemetryChannel final {
  std::string name;
  ChannelKind kind{ChannelKind::Custom};
  std::string unit;
  double rate_hz{0.0};
  std::string controller_variable;
};

// Camera attached to the cell, including its mounting frame and optional 3D data.
struct CameraConfig final {
  std::string name;
  std::string mounted_frame;        // CoordinateFrame name the camera is rigid to
  bool provides_depth{false};
  bool provides_point_cloud{false};
  bool hand_eye_calibrated{false};
  core::Pose3D extrinsics;          // camera pose in mounted_frame (hand-eye result)
  std::string intrinsics_ref;       // path/id of the intrinsics file
};

// Which motion primitives the controller accepts (capability advertisement).
struct MotionCapability final {
  MotionKind kind{MotionKind::MoveJ};
  bool supported{true};
  double default_speed{0.0};        // unit depends on kind
};

// Process parameters for welding passes; defaults that a plan can override.
struct WeldDefaults final {
  bool enabled{false};
  double travel_speed_mm_s{8.0};
  double segment_length_mm{2.0};
  double settle_delay_s{0.2};
  double tolerance_mm{0.5};
  std::string process_program;      // controller weld program / job id
};

struct MachineProfile final {
  int schema_version{1};
  std::string id;                   // stable identifier, e.g. "yaskawa_gp25_cell1"
  std::string display_name;
  std::string robot_model;          // "Yaskawa Motoman GP25"
  std::string controller;           // "YRC1000"
  std::string asset;                // visualization mesh, e.g. assets/.../gp25.glb

  std::vector<AxisSpec> axes;
  std::vector<CoordinateFrame> frames;
  std::vector<IoChannel> io;
  std::vector<TelemetryChannel> telemetry;
  std::vector<CameraConfig> cameras;
  std::vector<MotionCapability> motion;
  WeldDefaults weld;

  [[nodiscard]] std::size_t dof() const noexcept { return axes.size(); }

  [[nodiscard]] const CoordinateFrame* frame(std::string_view name) const noexcept {
    for (const auto& f : frames) {
      if (f.name == name) return &f;
    }
    return nullptr;
  }
};

}  // namespace cavr::machine
