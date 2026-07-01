#pragma once

// Deterministic controller used by tests, examples and the Studio demo. It is
// NOT a toy animation: it accepts a planned MotionTask (joint waypoints), builds
// a time schedule from the commanded speeds, and reports the resulting RobotState
// as a real controller would (program/servo state, current step, IO, weld arc,
// events). poll(now) advances the precomputed trajectory by wall-clock time.

#include <cavr/adapter_sdk/controller_adapter.hpp>
#include <cavr/machine/frames.hpp>
#include <cavr/machine/ik.hpp>
#include <cavr/machine/kinematics.hpp>
#include <cavr/machine/machine_profile.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

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

class MockController final : public sdk::ControllerAdapter {
 public:
  MockController() {
    // Tool 0 is the bare flange TCP (the GP25's 0.101 m tool plate), pre-calibrated
    // and selected — a real controller ships with its tools already calibrated.
    tools_.set_tool(0, core::Pose3D{core::Vec3{0.0, 0.0, 0.101}, core::Quaternion::identity()},
                    "flange TCP");
    tools_.select(0);
  }

  // The controller's tool table (10 slots), for selection and calibration.
  [[nodiscard]] machine::ToolTable* tools() override { return &tools_; }

  [[nodiscard]] sdk::ConnectResult connect(const sdk::ConnectionInfo& info) override {
    info_ = info;
    profile_ = make_gp25_profile();
    connected_ = true;
    return {true, {}};
  }
  void disconnect() override { connected_ = false; started_ = false; }
  [[nodiscard]] bool is_connected() const override { return connected_; }

  [[nodiscard]] machine::MachineProfile discover_profile() const override { return profile_; }

  [[nodiscard]] bool load_task(const machine::MotionTask& task) override {
    if (!connected_) return false;
    task_ = task;
    rebuild_schedule();
    state_ = machine::ProgramState::Loaded;
    return true;
  }

  [[nodiscard]] bool start() override {
    if (!connected_ || waypoints_.size() < 2) return false;
    started_ = true;
    paused_ = false;
    started_clock_ = false;  // captured on first poll
    last_step_ = -1;
    completed_emitted_ = false;
    state_ = machine::ProgramState::Running;
    return true;
  }
  void pause() override { if (started_) { paused_ = true; state_ = machine::ProgramState::Paused; } }
  void resume() override { if (started_) { paused_ = false; state_ = machine::ProgramState::Running; } }
  void stop() override { started_ = false; paused_ = false; state_ = machine::ProgramState::Aborted; }

  // Immediate jog: interrupt whatever is running and move from the current pose
  // to the commanded target (the scene -> robot direction). A joint target moves
  // there directly; a Cartesian (pose) target is solved through inverse
  // kinematics first, so MoveL-style jogging works too.
  [[nodiscard]] bool move_to(const machine::MotionCommand& command) override {
    if (!connected_) return false;
    std::vector<double> start = last_joints_.empty() ? std::vector<double>(dof(), 0.0) : last_joints_;
    start.resize(dof(), 0.0);

    const core::Pose3D tool = tools_.current_offset();
    std::vector<double> target;
    bool cartesian = false;
    if (command.target.joints) {
      target = *command.target.joints;
    } else if (command.target.pose) {
      const machine::IkResult ik =
          machine::inverse_kinematics(profile_.axes, *command.target.pose, start, tool);
      if (!ik.converged) return false;  // unreachable Cartesian target
      target = ik.joints;
      cartesian = true;
    } else {
      return false;
    }
    target.resize(dof(), 0.0);

    // Duration: a Cartesian move is timed by its TCP travel at the commanded
    // mm/s; a joint move by its largest axis sweep at the commanded rad/s.
    double dur;
    if (cartesian) {
      const core::Pose3D cur = forward_kinematics(profile_.axes, start, tool).tcp;
      const core::Vec3 tp = command.target.pose->position_m;
      const double dist_m = std::sqrt((tp.x_m - cur.position_m.x_m) * (tp.x_m - cur.position_m.x_m) +
                                      (tp.y_m - cur.position_m.y_m) * (tp.y_m - cur.position_m.y_m) +
                                      (tp.z_m - cur.position_m.z_m) * (tp.z_m - cur.position_m.z_m));
      const double speed_mm_s = command.speed > 0 ? command.speed : 50.0;
      dur = std::max(0.1, dist_m * 1000.0 / speed_mm_s);
    } else {
      double max_delta = 0.0;
      for (std::size_t i = 0; i < dof(); ++i) max_delta = std::max(max_delta, std::abs(target[i] - start[i]));
      const double speed = command.speed > 0 ? command.speed : deg(60);
      dur = std::max(0.1, max_delta / speed);
    }

    task_ = {command};
    waypoints_ = {std::move(start), std::move(target)};
    durations_ = {dur};
    weld_active_ = {0};
    starts_ = {0.0, dur};
    total_s_ = dur;
    state_ = machine::ProgramState::Running;
    started_ = true;
    paused_ = false;
    started_clock_ = false;
    last_step_ = -1;
    completed_emitted_ = false;
    return true;
  }

  [[nodiscard]] sdk::RobotState poll(core::Timestamp now) override {
    sdk::RobotState s;
    s.timestamp = now;
    s.servo_state = connected_ ? machine::ServoState::On : machine::ServoState::Off;
    s.program_state = state_;

    const std::int64_t now_ns = now.nanoseconds();
    if (!started_) {
      s.joint_positions = waypoints_.empty() ? std::vector<double>(dof(), 0.0) : waypoints_.front();
      finish_frame(s, 0, 0.0);
      return s;
    }
    if (!started_clock_) { start_ns_ = now_ns; last_ns_ = now_ns; started_clock_ = true; }
    if (paused_) {
      start_ns_ += (now_ns - last_ns_);  // freeze elapsed time
      last_ns_ = now_ns;
      s.joint_positions = sample_joints(frozen_t_, frozen_step_);
      finish_frame(s, frozen_step_, 0.0);
      return s;
    }
    last_ns_ = now_ns;
    const double t = static_cast<double>(now_ns - start_ns_) * 1e-9;
    frozen_t_ = t;

    int step = 0;
    double moving = 0.0;
    if (t >= total_s_) {
      s.joint_positions = waypoints_.back();
      step = static_cast<int>(task_.size()) - 1;
      state_ = machine::ProgramState::Completed;
      s.program_state = state_;
      if (!completed_emitted_) {
        s.events.push_back({now, machine::EventKind::ProgramCompleted, machine::Severity::Info, "Program completed"});
        completed_emitted_ = true;
      }
    } else {
      s.joint_positions = sample_joints(t, step);
      moving = is_motion_step(step) ? 1.0 : 0.0;
    }
    frozen_step_ = step;

    // step-boundary events
    if (step != last_step_ && state_ != machine::ProgramState::Completed) {
      if (last_step_ >= 0)
        s.events.push_back({now, machine::EventKind::StepCompleted, machine::Severity::Info,
                            "Step done: " + step_label(last_step_)});
      s.events.push_back({now, machine::EventKind::StepStarted, machine::Severity::Info,
                          "Step: " + step_label(step)});
      last_step_ = step;
    }
    finish_frame(s, step, moving);
    return s;
  }

  [[nodiscard]] const machine::MachineProfile& profile() const noexcept { return profile_; }

 private:
  [[nodiscard]] std::size_t dof() const noexcept { return profile_.axes.size(); }

  [[nodiscard]] std::string step_label(int i) const {
    if (i < 0 || i >= static_cast<int>(task_.size())) return "idle";
    const auto& c = task_[static_cast<std::size_t>(i)];
    return c.label.empty() ? machine::to_string(c.kind) : c.label;
  }

  [[nodiscard]] bool is_motion_step(int i) const {
    if (i < 0 || i >= static_cast<int>(task_.size())) return false;
    const auto k = task_[static_cast<std::size_t>(i)].kind;
    return k == machine::MotionKind::MoveJ || k == machine::MotionKind::MoveL || k == machine::MotionKind::MoveC;
  }

  void rebuild_schedule() {
    waypoints_.clear();
    durations_.clear();
    starts_.clear();
    weld_active_.clear();

    std::vector<double> current(dof(), 0.0);
    waypoints_.push_back(current);
    bool weld_on = false;
    for (const auto& cmd : task_) {
      std::vector<double> next = current;
      double dur = 0.1;
      if (cmd.kind == machine::MotionKind::MoveJ && cmd.target.joints) {
        next = *cmd.target.joints;
        next.resize(dof(), 0.0);
        double max_delta = 0.0;
        for (std::size_t i = 0; i < dof(); ++i) max_delta = std::max(max_delta, std::abs(next[i] - current[i]));
        const double speed = cmd.speed > 0 ? cmd.speed : deg(60);
        dur = std::max(0.1, max_delta / speed);
      } else if (cmd.kind == machine::MotionKind::MoveL || cmd.kind == machine::MotionKind::MoveC) {
        dur = 1.0;  // pose targets: real joints come from the controller
      } else if (cmd.kind == machine::MotionKind::Wait) {
        dur = std::max(0.0, cmd.wait_s);
      } else if (cmd.kind == machine::MotionKind::ToolOn) {
        weld_on = true; dur = 0.15;
      } else if (cmd.kind == machine::MotionKind::ToolOff) {
        weld_on = false; dur = 0.15;
      }
      durations_.push_back(dur);
      weld_active_.push_back(weld_on);
      waypoints_.push_back(next);
      current = next;
    }
    starts_.assign(durations_.size() + 1, 0.0);
    for (std::size_t i = 0; i < durations_.size(); ++i) starts_[i + 1] = starts_[i] + durations_[i];
    total_s_ = starts_.empty() ? 0.0 : starts_.back();
  }

  [[nodiscard]] std::vector<double> sample_joints(double t, int& step_out) const {
    if (waypoints_.size() < 2) return std::vector<double>(dof(), 0.0);
    std::size_t k = 0;
    while (k + 1 < starts_.size() && t >= starts_[k + 1]) ++k;
    if (k >= durations_.size()) k = durations_.size() - 1;
    step_out = static_cast<int>(k);
    const double dur = durations_[k];
    const double frac = dur > 1e-9 ? std::clamp((t - starts_[k]) / dur, 0.0, 1.0) : 1.0;
    std::vector<double> out(dof(), 0.0);
    for (std::size_t i = 0; i < dof(); ++i)
      out[i] = waypoints_[k][i] + (waypoints_[k + 1][i] - waypoints_[k][i]) * frac;
    return out;
  }

  void finish_frame(sdk::RobotState& s, int step, double moving) const {
    s.current_step = step;
    s.current_step_label = step_label(step);
    s.speed_fraction = moving;

    const auto fk = machine::forward_kinematics(profile_.axes, s.joint_positions, tools_.current_offset());
    s.tcp_pose = fk.tcp;

    const bool weld = step >= 0 && step < static_cast<int>(weld_active_.size()) && weld_active_[static_cast<std::size_t>(step)];
    s.io = {
        {"weld_on", weld ? 1.0 : 0.0},
        {"gas_on", weld ? 1.0 : 0.0},
        {"arc_established", weld ? 1.0 : 0.0},
        {"part_present", 1.0},
        {"wire_feed", weld ? 6.5 : 0.0},
    };
    s.tcp_speed_mm_s = moving > 0 ? profile_.weld.travel_speed_mm_s : 0.0;

    const bool scanning = s.current_step_label.find("scan") != std::string::npos;
    s.has_camera_frame = scanning;
    s.has_point_cloud = scanning;
    if (scanning) { s.camera_frame_id = "cam0"; s.point_cloud_id = "scan0"; }

    last_joints_ = s.joint_positions;  // remembered so a jog starts from the current pose
  }

  sdk::ConnectionInfo info_;
  machine::MachineProfile profile_;
  machine::MotionTask task_;
  machine::ProgramState state_{machine::ProgramState::Idle};

  std::vector<std::vector<double>> waypoints_;
  std::vector<double> durations_;
  std::vector<double> starts_;
  std::vector<char> weld_active_;
  double total_s_{0.0};
  mutable std::vector<double> last_joints_;  // last reported pose, so a jog starts from it
  machine::ToolTable tools_;                 // 10 tool slots; the selected one defines the TCP

  bool connected_{false};
  bool started_{false};
  bool paused_{false};
  bool started_clock_{false};
  bool completed_emitted_{false};
  int last_step_{-1};
  std::int64_t start_ns_{0};
  std::int64_t last_ns_{0};
  double frozen_t_{0.0};
  int frozen_step_{0};
};

}  // namespace cavr::adapters::mock_robot
