#pragma once

// Deterministic controller used by tests, examples and the Studio demo. It is a
// thin ControllerAdapter over a cavr::sim::VirtualRobot: the adapter owns the
// connection concern, and the VirtualRobot owns the actual robot behaviour —
// executing a motion::Trajectory (velocity profile + blending) against wall-clock
// time and reporting RobotState as a real controller would. The same VirtualRobot
// backs cavr-robotd, so "mock" here means "not a physical robot", not "a toy".

#include <cavr/adapter_sdk/controller_adapter.hpp>
#include <cavr/machine/machine_profile.hpp>
#include <cavr/motion/limits.hpp>
#include <cavr/sim/virtual_robot.hpp>

#include <cstddef>
#include <string>
#include <utility>

namespace cavr::adapters::mock_robot {

namespace machine = cavr::machine;
namespace sdk = cavr::adapter_sdk;

[[nodiscard]] inline double deg(double d) { return d * 3.14159265358979323846 / 180.0; }

// A representative Yaskawa GP25 cell profile (axes/frames/IO/cameras/weld).
[[nodiscard]] inline machine::MachineProfile make_gp25_profile() {
  using machine::AxisSpec;
  machine::MachineProfile p;
  p.schema_version = 1;
  p.id = "yaskawa_gp25_cell1";
  p.display_name = "GP25 Welding Cell 1";
  p.robot_model = "Yaskawa Motoman GP25";
  p.controller = "YRC1000";
  p.asset = "assets/robots/yaskawa_gp25/gp25.glb";

  p.axes = {
      {"S", machine::JointType::Revolute, {0, 1, 0}, {0.0, 0.169, 0.0}, deg(-180), deg(180), deg(210), "PULSE[1]"},
      {"L", machine::JointType::Revolute, {1, 0, 0}, {-0.157, 0.336, 0.150}, deg(-105), deg(155), deg(210), "PULSE[2]"},
      {"U", machine::JointType::Revolute, {1, 0, 0}, {0.157, 0.760, 0.0}, deg(-86), deg(160), deg(265), "PULSE[3]"},
      {"R", machine::JointType::Revolute, {0, 0, 1}, {0.0, 0.200, 0.302}, deg(-200), deg(200), deg(420), "PULSE[4]"},
      {"B", machine::JointType::Revolute, {1, 0, 0}, {0.0, 0.0, 0.493}, deg(-150), deg(150), deg(420), "PULSE[5]"},
      {"T", machine::JointType::Revolute, {0, 0, 1}, {0.0, 0.0, 0.0}, deg(-455), deg(455), deg(885), "PULSE[6]"},
  };

  p.frames = {
      {"world", machine::FrameKind::World, "", {}},
      {"base", machine::FrameKind::Base, "world", {}},
      {"flange", machine::FrameKind::Flange, "base", {}},
      {"tcp", machine::FrameKind::Tool, "flange", {core::Vec3{0.0, 0.0, 0.101}, core::Quaternion::identity()}},
      {"table", machine::FrameKind::User, "world", {core::Vec3{0.6, 0.0, 0.4}, core::Quaternion::identity()}},
      {"camera", machine::FrameKind::Camera, "flange", {}},
  };

  p.io = {
      {"weld_on", machine::IoKind::Digital, machine::IoDirection::Output, 1, "DOUT[1]"},
      {"gas_on", machine::IoKind::Digital, machine::IoDirection::Output, 2, "DOUT[2]"},
      {"arc_established", machine::IoKind::Digital, machine::IoDirection::Input, 1, "DIN[1]"},
      {"part_present", machine::IoKind::Digital, machine::IoDirection::Input, 2, "DIN[2]"},
      {"wire_feed", machine::IoKind::Analog, machine::IoDirection::Output, 1, "AOUT[1]"},
  };

  p.telemetry = {
      {"joint_position", machine::ChannelKind::JointPosition, "rad", 125.0, "RPOS"},
      {"cartesian_pose", machine::ChannelKind::CartesianPose, "m", 125.0, "RCART"},
      {"speed", machine::ChannelKind::Speed, "mm/s", 125.0, "SPEED"},
      {"program_state", machine::ChannelKind::ProgramState, "", 50.0, "PSTATE"},
      {"io_state", machine::ChannelKind::IoState, "", 50.0, "IO"},
      {"error", machine::ChannelKind::Error, "", 50.0, "ALARM"},
      {"event", machine::ChannelKind::Event, "", 50.0, "EVENT"},
      {"camera_image", machine::ChannelKind::CameraImage, "", 30.0, "CAM0/rgb"},
      {"depth_image", machine::ChannelKind::DepthImage, "", 30.0, "CAM0/depth"},
      {"point_cloud", machine::ChannelKind::PointCloud, "", 10.0, "CAM0/points"},
  };

  machine::CameraConfig cam;
  cam.name = "wrist_3d";
  cam.mounted_frame = "camera";
  cam.provides_depth = true;
  cam.provides_point_cloud = true;
  cam.hand_eye_calibrated = true;
  cam.extrinsics = {core::Vec3{0.05, 0.0, 0.08}, core::Quaternion::identity()};
  cam.intrinsics_ref = "calibration/cam0_intrinsics.yaml";
  p.cameras = {cam};

  p.motion = {
      {machine::MotionKind::MoveJ, true, deg(60)},
      {machine::MotionKind::MoveL, true, 250.0},
      {machine::MotionKind::MoveC, true, 250.0},
      {machine::MotionKind::Wait, true, 0.0},
      {machine::MotionKind::ToolOn, true, 0.0},
      {machine::MotionKind::ToolOff, true, 0.0},
  };

  p.weld.enabled = true;
  p.weld.travel_speed_mm_s = 8.0;
  p.weld.segment_length_mm = 2.0;
  p.weld.settle_delay_s = 0.2;
  p.weld.tolerance_mm = 0.5;
  p.weld.process_program = "WELD_JOB_12";
  return p;
}

// A representative PNR 6-axis articulated robot. Same base-frame convention as the
// GP25 (Y up, origins relative to the parent joint) so forward/inverse kinematics
// work unchanged, but with PNR's IO vocabulary: the Y/M/AIN/AOT/GIN/GOT banks a
// PNR controller exposes, mapped onto controller_variable. It reuses the GP25 mesh
// for visualization until a PNR asset is added. This is the first proof that the
// registry is vendor-neutral: a different profile, the same MockController.
[[nodiscard]] inline machine::MachineProfile make_pnr_profile() {
  machine::MachineProfile p;
  p.schema_version = 1;
  p.id = "pnr_6axis_cell1";
  p.display_name = "PNR 6-Axis Cell 1";
  p.robot_model = "PNR PR6-900";
  p.controller = "PNR-C";
  p.asset = "assets/robots/yaskawa_gp25/gp25.glb";  // placeholder mesh until a PNR asset exists

  p.axes = {
      {"J1", machine::JointType::Revolute, {0, 1, 0}, {0.0, 0.150, 0.0}, deg(-170), deg(170), deg(230), "PULSE[1]"},
      {"J2", machine::JointType::Revolute, {1, 0, 0}, {-0.140, 0.190, 0.050}, deg(-90), deg(135), deg(225), "PULSE[2]"},
      {"J3", machine::JointType::Revolute, {1, 0, 0}, {0.140, 0.420, 0.0}, deg(-180), deg(70), deg(230), "PULSE[3]"},
      {"J4", machine::JointType::Revolute, {0, 0, 1}, {0.0, 0.150, 0.300}, deg(-190), deg(190), deg(430), "PULSE[4]"},
      {"J5", machine::JointType::Revolute, {1, 0, 0}, {0.0, 0.0, 0.240}, deg(-120), deg(120), deg(430), "PULSE[5]"},
      {"J6", machine::JointType::Revolute, {0, 0, 1}, {0.0, 0.0, 0.0}, deg(-360), deg(360), deg(630), "PULSE[6]"},
  };

  p.frames = {
      {"world", machine::FrameKind::World, "", {}},
      {"base", machine::FrameKind::Base, "world", {}},
      {"flange", machine::FrameKind::Flange, "base", {}},
      {"tcp", machine::FrameKind::Tool, "flange", {core::Vec3{0.0, 0.0, 0.090}, core::Quaternion::identity()}},
      {"table", machine::FrameKind::User, "world", {core::Vec3{0.5, 0.0, 0.35}, core::Quaternion::identity()}},
      {"camera", machine::FrameKind::Camera, "flange", {}},
  };

  // PNR IO banks: Y = digital outputs, M = internal relays (merkers), AIN/AOT =
  // analog in/out, GIN/GOT = group (word) in/out.
  p.io = {
      {"Y0", machine::IoKind::Digital, machine::IoDirection::Output, 0, "Y0"},
      {"Y1", machine::IoKind::Digital, machine::IoDirection::Output, 1, "Y1"},
      {"X0", machine::IoKind::Digital, machine::IoDirection::Input, 0, "X0"},
      {"X1", machine::IoKind::Digital, machine::IoDirection::Input, 1, "X1"},
      {"M0", machine::IoKind::Digital, machine::IoDirection::Internal, 0, "M0"},
      {"M1", machine::IoKind::Digital, machine::IoDirection::Internal, 1, "M1"},
      {"AIN0", machine::IoKind::Analog, machine::IoDirection::Input, 0, "AIN0"},
      {"AOT0", machine::IoKind::Analog, machine::IoDirection::Output, 0, "AOT0"},
      {"GIN0", machine::IoKind::Group, machine::IoDirection::Input, 0, "GIN0"},
      {"GOT0", machine::IoKind::Group, machine::IoDirection::Output, 0, "GOT0"},
  };

  p.telemetry = {
      {"joint_position", machine::ChannelKind::JointPosition, "rad", 100.0, "RPOS"},
      {"cartesian_pose", machine::ChannelKind::CartesianPose, "m", 100.0, "RCART"},
      {"speed", machine::ChannelKind::Speed, "mm/s", 100.0, "SPEED"},
      {"program_state", machine::ChannelKind::ProgramState, "", 50.0, "PSTATE"},
      {"io_state", machine::ChannelKind::IoState, "", 50.0, "IO"},
      {"error", machine::ChannelKind::Error, "", 50.0, "ALARM"},
      {"event", machine::ChannelKind::Event, "", 50.0, "EVENT"},
  };

  p.motion = {
      {machine::MotionKind::MoveJ, true, deg(60)},
      {machine::MotionKind::MoveL, true, 250.0},
      {machine::MotionKind::MoveC, true, 250.0},
      {machine::MotionKind::Wait, true, 0.0},
      {machine::MotionKind::ToolOn, true, 0.0},
      {machine::MotionKind::ToolOff, true, 0.0},
  };

  p.weld.enabled = false;  // a general-purpose PNR arm, not a welding cell
  return p;
}


// A thin ControllerAdapter over cavr::sim::VirtualRobot. The adapter owns the
// connection lifecycle and forwards everything else to the VirtualRobot, which is
// where the robot's behaviour (trajectory execution, IO, weld, events) actually
// lives. Tool 0 is pre-calibrated to the profile's tool frame — a real controller
// ships with its tools already set (done in the VirtualRobot constructor).
class MockController final : public sdk::ControllerAdapter {
 public:
  MockController() : MockController(make_gp25_profile()) {}

  // Serve a specific profile (e.g. make_pnr_profile()), so one mock backend can
  // stand in for any robot in the registry. An optional MotionLimits selects the
  // velocity-profile shape (trapezoidal by default; S-curve for gentler motion).
  explicit MockController(machine::MachineProfile profile,
                          motion::MotionLimits limits = motion::MotionLimits::trapezoidal())
      : robot_(std::move(profile), limits) {}

  // The controller's tool table (10 slots), for selection and calibration.
  [[nodiscard]] machine::ToolTable* tools() override { return &robot_.tools(); }
  [[nodiscard]] bool select_tool(int slot) override { return robot_.tools().select(slot); }
  [[nodiscard]] bool calibrate_tool(int slot, const core::Pose3D& tcp_offset) override {
    robot_.tools().set_tool(static_cast<std::size_t>(slot), tcp_offset);
    return true;
  }
  [[nodiscard]] bool clear_tool(int slot) override {
    robot_.tools().clear_tool(static_cast<std::size_t>(slot));
    return true;
  }

  // Write an IO channel. Only output/internal channels are writable; a write to an
  // input or an unknown channel is rejected.
  [[nodiscard]] bool write_io(const std::string& name, double value) override {
    return robot_.write_io(name, value);
  }

  [[nodiscard]] sdk::ConnectResult connect(const sdk::ConnectionInfo& info) override {
    info_ = info;
    robot_.seed_io();       // fresh IO state on (re)connect
    robot_.set_servo(true);
    connected_ = true;
    return {true, {}};
  }
  void disconnect() override {
    connected_ = false;
    robot_.set_servo(false);
    robot_.halt();
  }
  [[nodiscard]] bool is_connected() const override { return connected_; }

  [[nodiscard]] machine::MachineProfile discover_profile() const override { return robot_.profile(); }

  [[nodiscard]] bool load_task(const machine::MotionTask& task) override {
    return connected_ && robot_.load_task(task);
  }

  [[nodiscard]] bool start() override { return connected_ && robot_.start(); }
  void pause() override { robot_.pause(); }
  void resume() override { robot_.resume(); }
  void stop() override { robot_.stop(); }

  // Immediate jog outside the loaded program (the scene -> robot direction).
  [[nodiscard]] bool move_to(const machine::MotionCommand& command) override {
    return connected_ && robot_.move_to(command);
  }

  [[nodiscard]] sdk::RobotState poll(core::Timestamp now) override { return robot_.poll(now); }

  [[nodiscard]] const machine::MachineProfile& profile() const noexcept { return robot_.profile(); }

  // The underlying virtual robot, for advanced use (e.g. tuning motion limits).
  [[nodiscard]] cavr::sim::VirtualRobot& virtual_robot() noexcept { return robot_; }

 private:
  cavr::sim::VirtualRobot robot_;
  sdk::ConnectionInfo info_;
  bool connected_{false};
};

}  // namespace cavr::adapters::mock_robot
