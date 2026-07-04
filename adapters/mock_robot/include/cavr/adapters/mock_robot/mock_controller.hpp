#pragma once

// Deterministic controller used by tests, examples and the Studio demo. It is
// NOT a toy animation: it accepts a planned MotionTask (joint waypoints), builds
// a time schedule from the commanded speeds, and reports the resulting RobotState
// as a real controller would (program/servo state, current step, IO, weld arc,
// events). poll(now) advances the precomputed trajectory by wall-clock time.

#include <cavr/adapter_sdk/controller_adapter.hpp>
#include <cavr/machine/arc.hpp>
#include <cavr/machine/frames.hpp>
#include <cavr/machine/ik.hpp>
#include <cavr/machine/kinematics.hpp>
#include <cavr/machine/machine_profile.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
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

class MockController final : public sdk::ControllerAdapter {
 public:
  MockController() : MockController(make_gp25_profile()) {}

  // Serve a specific profile (e.g. make_pnr_profile()), so one mock backend can
  // stand in for any robot in the registry. Tool 0 is pre-calibrated to the
  // profile's tool frame — a real controller ships with its tools already set.
  explicit MockController(machine::MachineProfile profile)
      : custom_profile_(std::move(profile)) {
    core::Pose3D flange_tcp{core::Vec3{0.0, 0.0, 0.101}, core::Quaternion::identity()};
    if (const machine::CoordinateFrame* tcp = custom_profile_.frame("tcp")) flange_tcp = tcp->transform;
    tools_.set_tool(0, flange_tcp, "flange TCP");
    tools_.select(0);
  }

  // The controller's tool table (10 slots), for selection and calibration.
  [[nodiscard]] machine::ToolTable* tools() override { return &tools_; }
  [[nodiscard]] bool select_tool(int slot) override { return tools_.select(slot); }
  [[nodiscard]] bool calibrate_tool(int slot, const core::Pose3D& tcp_offset) override {
    tools_.set_tool(static_cast<std::size_t>(slot), tcp_offset);
    return true;
  }
  [[nodiscard]] bool clear_tool(int slot) override {
    tools_.clear_tool(static_cast<std::size_t>(slot));
    return true;
  }

  // Write an IO channel. Only output/internal channels are writable; a write to an
  // input or an unknown channel is rejected. The value is held and reported back in
  // the telemetry stream, so the scene reflects it on the next poll.
  [[nodiscard]] bool write_io(const std::string& name, double value) override {
    const auto it = std::find_if(profile_.io.begin(), profile_.io.end(),
                                 [&](const machine::IoChannel& c) { return c.name == name; });
    if (it == profile_.io.end() || it->direction == machine::IoDirection::Input) return false;
    io_[name] = value;
    return true;
  }

  [[nodiscard]] sdk::ConnectResult connect(const sdk::ConnectionInfo& info) override {
    info_ = info;
    profile_ = custom_profile_;
    // Seed the IO map from the profile: every channel starts at 0, except a
    // "part_present" input which the cell reports as present (a GP25 demo signal).
    io_.clear();
    for (const auto& c : profile_.io) io_[c.name] = 0.0;
    if (io_.count("part_present")) io_["part_present"] = 1.0;
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

    // MoveC: follow a circular arc start -> via -> target. Sample the arc, solve IK
    // for each pose (seeded from the previous one), and run the resulting joint
    // knots as a multi-segment trajectory, so the TCP traces the arc, not a chord.
    if (command.kind == machine::MotionKind::MoveC && command.via && command.target.pose) {
      const core::Pose3D start_pose = machine::forward_kinematics(profile_.axes, start, tool).tcp;
      constexpr int kArcSamples = 13;
      const machine::ArcPath arc =
          machine::sample_arc(start_pose, *command.via, *command.target.pose, kArcSamples);

      std::vector<std::vector<double>> knots;
      knots.reserve(arc.poses.size());
      knots.push_back(start);
      std::vector<double> seed = start;
      for (std::size_t i = 1; i < arc.poses.size(); ++i) {
        const machine::IkResult ik =
            machine::inverse_kinematics(profile_.axes, arc.poses[i], seed, tool);
        if (!ik.converged) return false;  // the arc leaves the reachable workspace
        std::vector<double> q = ik.joints;
        q.resize(dof(), 0.0);
        knots.push_back(q);
        seed = std::move(q);
      }

      // Each sub-segment is timed by its chord length at the commanded mm/s.
      const double speed_mm_s = command.speed > 0 ? command.speed : 50.0;
      durations_.clear();
      double total = 0.0;
      for (std::size_t i = 1; i < arc.poses.size(); ++i) {
        const core::Vec3 p0 = arc.poses[i - 1].position_m;
        const core::Vec3 p1 = arc.poses[i].position_m;
        const double seg_m = std::sqrt((p1.x_m - p0.x_m) * (p1.x_m - p0.x_m) +
                                       (p1.y_m - p0.y_m) * (p1.y_m - p0.y_m) +
                                       (p1.z_m - p0.z_m) * (p1.z_m - p0.z_m));
        const double d = std::max(0.02, seg_m * 1000.0 / speed_mm_s);
        durations_.push_back(d);
        total += d;
      }

      const bool weld = command.weld && command.weld->enabled;
      task_ = {command};                              // one program step (the arc)
      seg_cmd_.assign(knots.size() - 1, 0);           // every sub-segment maps to it
      weld_active_.assign(knots.size() - 1, static_cast<char>(weld ? 1 : 0));
      waypoints_ = std::move(knots);
      starts_.assign(durations_.size() + 1, 0.0);
      for (std::size_t i = 0; i < durations_.size(); ++i) starts_[i + 1] = starts_[i] + durations_[i];
      total_s_ = total;
      state_ = machine::ProgramState::Running;
      started_ = true;
      paused_ = false;
      started_clock_ = false;
      last_step_ = -1;
      completed_emitted_ = false;
      return true;
    }

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
    seg_cmd_ = {0};
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
      finish_frame(s, 0, 0, 0.0);
      return s;
    }
    if (!started_clock_) { start_ns_ = now_ns; last_ns_ = now_ns; started_clock_ = true; }
    if (paused_) {
      start_ns_ += (now_ns - last_ns_);  // freeze elapsed time
      last_ns_ = now_ns;
      s.joint_positions = sample_joints(frozen_t_, frozen_step_);
      finish_frame(s, seg_to_cmd(frozen_step_), frozen_step_, 0.0);
      return s;
    }
    last_ns_ = now_ns;
    const double t = static_cast<double>(now_ns - start_ns_) * 1e-9;
    frozen_t_ = t;

    int seg = 0;
    double moving = 0.0;
    if (t >= total_s_) {
      s.joint_positions = waypoints_.back();
      seg = static_cast<int>(durations_.size()) - 1;  // last schedule segment
      state_ = machine::ProgramState::Completed;
      s.program_state = state_;
      if (!completed_emitted_) {
        s.events.push_back({now, machine::EventKind::ProgramCompleted, machine::Severity::Info, "Program completed"});
        completed_emitted_ = true;
      }
    } else {
      s.joint_positions = sample_joints(t, seg);
      moving = is_motion_step(seg_to_cmd(seg)) ? 1.0 : 0.0;
    }
    frozen_step_ = seg;
    const int cmd = seg_to_cmd(seg);

    // step-boundary events fire per command / program line, not per arc sub-segment.
    if (cmd != last_step_ && state_ != machine::ProgramState::Completed) {
      if (last_step_ >= 0)
        s.events.push_back({now, machine::EventKind::StepCompleted, machine::Severity::Info,
                            "Step done: " + step_label(last_step_)});
      s.events.push_back({now, machine::EventKind::StepStarted, machine::Severity::Info,
                          "Step: " + step_label(cmd)});
      last_step_ = cmd;
    }
    finish_frame(s, cmd, seg, moving);
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

  // Map a schedule segment to the task command it belongs to (a MoveC expands to
  // several segments that all share one command / program line).
  [[nodiscard]] int seg_to_cmd(int seg) const {
    if (seg >= 0 && seg < static_cast<int>(seg_cmd_.size())) return seg_cmd_[static_cast<std::size_t>(seg)];
    return static_cast<int>(task_.size()) - 1;
  }

  [[nodiscard]] static double cart_dist_m(const core::Vec3& a, const core::Vec3& b) {
    return std::sqrt((a.x_m - b.x_m) * (a.x_m - b.x_m) + (a.y_m - b.y_m) * (a.y_m - b.y_m) +
                     (a.z_m - b.z_m) * (a.z_m - b.z_m));
  }

  void rebuild_schedule() {
    waypoints_.clear();
    durations_.clear();
    starts_.clear();
    weld_active_.clear();
    seg_cmd_.clear();

    const core::Pose3D tool = tools_.current_offset();
    std::vector<double> current(dof(), 0.0);
    waypoints_.push_back(current);
    bool weld_on = false;

    // Append one schedule segment ending at `next`, taking `dur` seconds, belonging
    // to command `ci`.
    const auto push_seg = [&](const std::vector<double>& next, double dur, std::size_t ci) {
      durations_.push_back(dur);
      weld_active_.push_back(static_cast<char>(weld_on ? 1 : 0));
      waypoints_.push_back(next);
      seg_cmd_.push_back(static_cast<int>(ci));
    };

    for (std::size_t ci = 0; ci < task_.size(); ++ci) {
      const auto& cmd = task_[ci];
      if (cmd.kind == machine::MotionKind::MoveJ && cmd.target.joints) {
        std::vector<double> next = *cmd.target.joints;
        next.resize(dof(), 0.0);
        double max_delta = 0.0;
        for (std::size_t i = 0; i < dof(); ++i) max_delta = std::max(max_delta, std::abs(next[i] - current[i]));
        const double speed = cmd.speed > 0 ? cmd.speed : deg(60);
        push_seg(next, std::max(0.1, max_delta / speed), ci);
        current = next;
      } else if (cmd.kind == machine::MotionKind::MoveL && cmd.target.pose) {
        // Straight-line Cartesian move: IK the target from the current pose.
        const machine::IkResult ik = machine::inverse_kinematics(profile_.axes, *cmd.target.pose, current, tool);
        std::vector<double> next = ik.converged ? ik.joints : current;
        next.resize(dof(), 0.0);
        const core::Pose3D cur_pose = machine::forward_kinematics(profile_.axes, current, tool).tcp;
        const double dist_m = cart_dist_m(cmd.target.pose->position_m, cur_pose.position_m);
        const double speed_mm_s = cmd.speed > 0 ? cmd.speed : 50.0;
        push_seg(next, std::max(0.1, dist_m * 1000.0 / speed_mm_s), ci);
        current = next;
      } else if (cmd.kind == machine::MotionKind::MoveC && cmd.target.pose) {
        // Circular arc start -> via -> end, expanded into sub-segments (all sharing
        // this command index) with IK along the path.
        const core::Pose3D start_pose = machine::forward_kinematics(profile_.axes, current, tool).tcp;
        const core::Pose3D via = cmd.via ? *cmd.via : start_pose;
        const machine::ArcPath arc = machine::sample_arc(start_pose, via, *cmd.target.pose, 13);
        const double speed_mm_s = cmd.speed > 0 ? cmd.speed : 50.0;
        std::vector<double> seed = current;
        for (std::size_t i = 1; i < arc.poses.size(); ++i) {
          const machine::IkResult ik = machine::inverse_kinematics(profile_.axes, arc.poses[i], seed, tool);
          std::vector<double> next = ik.converged ? ik.joints : seed;
          next.resize(dof(), 0.0);
          const double seg_m = cart_dist_m(arc.poses[i].position_m, arc.poses[i - 1].position_m);
          push_seg(next, std::max(0.02, seg_m * 1000.0 / speed_mm_s), ci);
          seed = std::move(next);
        }
        current = seed;
      } else if (cmd.kind == machine::MotionKind::Wait) {
        push_seg(current, std::max(0.0, cmd.wait_s), ci);
      } else if (cmd.kind == machine::MotionKind::ToolOn) {
        weld_on = true;
        push_seg(current, 0.15, ci);
      } else if (cmd.kind == machine::MotionKind::ToolOff) {
        weld_on = false;
        push_seg(current, 0.15, ci);
      } else {
        push_seg(current, 0.1, ci);  // unknown / no-op step
      }
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

  void finish_frame(sdk::RobotState& s, int cmd, int seg, double moving) const {
    s.current_step = cmd;
    s.current_step_label = step_label(cmd);
    s.speed_fraction = moving;

    const auto fk = machine::forward_kinematics(profile_.axes, s.joint_positions, tools_.current_offset());
    s.tcp_pose = fk.tcp;

    // The GP25 weld process drives its own signals while welding; other channels
    // (and every channel on a non-welding robot like the PNR) hold whatever was
    // last written through write_io. Report the IO map in the profile's order.
    const bool weld = seg >= 0 && seg < static_cast<int>(weld_active_.size()) && weld_active_[static_cast<std::size_t>(seg)];
    if (io_.count("weld_on")) {
      io_["weld_on"] = weld ? 1.0 : 0.0;
      io_["gas_on"] = weld ? 1.0 : 0.0;
      io_["arc_established"] = weld ? 1.0 : 0.0;
      if (io_.count("wire_feed")) io_["wire_feed"] = weld ? 6.5 : 0.0;
    }
    s.io.clear();
    s.io.reserve(profile_.io.size());
    for (const auto& c : profile_.io) s.io.push_back({c.name, io_[c.name]});
    s.tcp_speed_mm_s = moving > 0 ? profile_.weld.travel_speed_mm_s : 0.0;

    const bool scanning = s.current_step_label.find("scan") != std::string::npos;
    s.has_camera_frame = scanning;
    s.has_point_cloud = scanning;
    if (scanning) { s.camera_frame_id = "cam0"; s.point_cloud_id = "scan0"; }

    last_joints_ = s.joint_positions;  // remembered so a jog starts from the current pose
  }

  sdk::ConnectionInfo info_;
  machine::MachineProfile custom_profile_;  // the profile this mock serves on connect()
  machine::MachineProfile profile_;
  machine::MotionTask task_;
  machine::ProgramState state_{machine::ProgramState::Idle};

  std::vector<std::vector<double>> waypoints_;
  std::vector<double> durations_;
  std::vector<double> starts_;
  std::vector<char> weld_active_;
  std::vector<int> seg_cmd_;  // schedule segment -> task command index (MoveC expands to many)
  double total_s_{0.0};
  mutable std::vector<double> last_joints_;  // last reported pose, so a jog starts from it
  mutable std::map<std::string, double> io_;  // live IO values (written + process-driven)
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
