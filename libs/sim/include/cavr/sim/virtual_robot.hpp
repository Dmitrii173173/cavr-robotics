#pragma once

// VirtualRobot: the stateful simulation of one robot. It owns the joint state, the
// tool table and the IO map, executes a planned Trajectory against wall-clock time,
// drives process signals (weld) and emits controller events — and reports all of it
// as a RobotState, exactly as a real controller's telemetry would.
//
// It has no Qt, no sockets and no notion of "connection": that belongs to the
// adapter on top (MockController for Studio, and the same backend inside
// cavr-robotd). Extracting it means motion can be driven and inspected in a test
// without an adapter, and the realism knobs (velocity profile now; collisions,
// dynamics, faults later) have one home instead of bloating the mock.
//
// The trajectory itself is built by libs/motion::plan_task — the same planner the
// validator uses — so what is checked and what runs are one code path.

#include <cavr/adapter_sdk/robot_state.hpp>
#include <cavr/core/time.hpp>
#include <cavr/fault_injection/fault_model.hpp>
#include <cavr/machine/frames.hpp>
#include <cavr/machine/kinematics.hpp>
#include <cavr/machine/machine_profile.hpp>
#include <cavr/machine/motion.hpp>
#include <cavr/motion/limits.hpp>
#include <cavr/motion/plan_task.hpp>

#include <algorithm>
#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace cavr::sim {

namespace machine = cavr::machine;
namespace motion = cavr::motion;
namespace fault = cavr::fault_injection;
namespace sdk = cavr::adapter_sdk;

class VirtualRobot final {
 public:
  explicit VirtualRobot(machine::MachineProfile profile,
                        motion::MotionLimits limits = motion::MotionLimits::trapezoidal())
      : profile_(std::move(profile)), limits_(limits) {
    core::Pose3D flange_tcp{core::Vec3{0.0, 0.0, 0.101}, core::Quaternion::identity()};
    if (const machine::CoordinateFrame* tcp = profile_.frame("tcp")) flange_tcp = tcp->transform;
    tools_.set_tool(0, flange_tcp, "flange TCP");
    (void)tools_.select(0);
    seed_io();
  }

  [[nodiscard]] const machine::MachineProfile& profile() const noexcept { return profile_; }
  [[nodiscard]] machine::ToolTable& tools() noexcept { return tools_; }
  [[nodiscard]] std::size_t dof() const noexcept { return profile_.axes.size(); }
  [[nodiscard]] machine::ProgramState state() const noexcept { return state_; }
  [[nodiscard]] const motion::MotionLimits& limits() const noexcept { return limits_; }
  void set_limits(motion::MotionLimits limits) noexcept { limits_ = limits; }

  // The fault scenario driving this robot. Faults are re-armed on each start(), so
  // configuring the injector between runs replays deterministically.
  [[nodiscard]] fault::FaultInjector& faults() noexcept { return faults_; }
  [[nodiscard]] const fault::FaultInjector& faults() const noexcept { return faults_; }
  [[nodiscard]] bool faulted() const noexcept { return faulted_; }

  // Clear a latched fault (E-stop / servo / arc loss) and re-arm the scenario — the
  // operator's "reset" after acknowledging an alarm. Does not remove the specs.
  void clear_fault() {
    faulted_ = false;
    arc_fault_ = false;
    fault_severity_ = machine::Severity::Info;
    fault_code_ = 0;
    fault_message_.clear();
    faults_.arm();
  }

  // Servo power: the adapter energises the robot on connect and de-energises on
  // disconnect. Off is reported as ServoState::Off in telemetry.
  void set_servo(bool on) noexcept { servo_on_ = on; }

  // Reset IO to the profile's channels (all 0), with a GP25-style part-present
  // input asserted so the demo cell reports a part in the fixture.
  void seed_io() {
    io_.clear();
    for (const auto& c : profile_.io) io_[c.name] = 0.0;
    if (io_.count("part_present")) io_["part_present"] = 1.0;
  }

  [[nodiscard]] bool write_io(const std::string& name, double value) {
    const auto it = std::find_if(profile_.io.begin(), profile_.io.end(),
                                 [&](const machine::IoChannel& c) { return c.name == name; });
    if (it == profile_.io.end() || it->direction == machine::IoDirection::Input) return false;
    io_[name] = value;
    return true;
  }

  // Load a program, planned from home (all-zero joints) like a controller that
  // starts a job from its recorded start pose.
  [[nodiscard]] bool load_task(const machine::MotionTask& task) {
    task_ = task;
    plan_ = motion::plan_task(profile_.axes, tools_.current_offset(), task_,
                              std::vector<double>(dof(), 0.0), limits_);
    state_ = machine::ProgramState::Loaded;
    return true;
  }

  [[nodiscard]] bool start() {
    if (plan_.trajectory.empty()) return false;
    begin_execution();
    return true;
  }

  void pause() {
    if (running_) { paused_ = true; state_ = machine::ProgramState::Paused; }
  }
  void resume() {
    if (running_) { paused_ = false; state_ = machine::ProgramState::Running; }
  }
  void stop() { running_ = false; paused_ = false; state_ = machine::ProgramState::Aborted; }

  // Stop advancing without marking the program aborted — used when the adapter
  // disconnects (the robot simply stops being driven).
  void halt() noexcept { running_ = false; paused_ = false; }

  // Immediate jog: plan a one-command trajectory from the current pose and run it,
  // interrupting whatever program was loaded. A Cartesian/arc target that leaves
  // the workspace is rejected (returns false), matching a real controller.
  [[nodiscard]] bool move_to(const machine::MotionCommand& command) {
    std::vector<double> start = last_joints_.empty() ? std::vector<double>(dof(), 0.0) : last_joints_;
    start.resize(dof(), 0.0);
    task_ = {command};
    motion::TaskPlan plan =
        motion::plan_task(profile_.axes, tools_.current_offset(), task_, start, limits_);
    if (!plan.reachable) return false;
    plan_ = std::move(plan);
    begin_execution();
    return true;
  }

  [[nodiscard]] sdk::RobotState poll(core::Timestamp now) {
    sdk::RobotState s;
    s.timestamp = now;
    s.program_state = state_;

    const std::int64_t now_ns = now.nanoseconds();

    // A latched hard fault (E-stop / servo) holds the robot at the pose it stopped
    // in and reports the alarm on every frame until the operator clears it.
    if (faulted_) {
      s.joint_positions = plan_.trajectory.empty() ? home_pose() : plan_.trajectory.sample(frozen_t_);
      finish_frame(s, command_of(frozen_t_), 0.0);
      apply_fault_fields(s);
      return s;
    }
    if (!running_) {
      s.joint_positions = home_pose();
      finish_frame(s, 0, 0.0);
      apply_fault_fields(s);
      return s;
    }
    if (!started_clock_) { start_ns_ = now_ns; last_ns_ = now_ns; started_clock_ = true; }
    if (paused_) {
      start_ns_ += (now_ns - last_ns_);  // freeze elapsed time
      last_ns_ = now_ns;
      s.joint_positions = plan_.trajectory.sample(frozen_t_);
      finish_frame(s, command_of(frozen_t_), 0.0);
      apply_fault_fields(s);
      return s;
    }
    last_ns_ = now_ns;
    const double t = static_cast<double>(now_ns - start_ns_) * 1e-9;
    frozen_t_ = t;

    const double total = plan_.trajectory.duration();
    double moving = 0.0;
    int cmd = 0;
    if (t >= total) {
      s.joint_positions = plan_.trajectory.last_knot();
      if (s.joint_positions.empty()) s.joint_positions = home_pose();
      cmd = command_of(total);
      state_ = machine::ProgramState::Completed;
      s.program_state = state_;
      if (!completed_emitted_) {
        s.events.push_back({now, machine::EventKind::ProgramCompleted, machine::Severity::Info,
                            "Program completed"});
        completed_emitted_ = true;
      }
    } else {
      s.joint_positions = plan_.trajectory.sample(t);
      const int seg = plan_.trajectory.segment_at(t);
      cmd = command_of(t);
      moving = plan_.trajectory.segment_is_motion(seg) ? 1.0 : 0.0;

      // Evaluate the fault scenario against this tick. A hard fault aborts the run
      // here; a soft fault (arc loss) latches but the motion continues.
      const double dt = std::max(0.0, t - prev_elapsed_);
      for (const fault::FaultTrip& trip : faults_.poll(t, dt, cmd, prev_fault_step_)) {
        apply_trip(trip, s, now);
      }
      prev_elapsed_ = t;
      prev_fault_step_ = cmd;
      if (faulted_) moving = 0.0;  // aborted this tick: hold, do not report motion
    }

    // Step-boundary events fire per command / program line, not per sub-segment.
    if (cmd != last_step_ && state_ == machine::ProgramState::Running) {
      if (last_step_ >= 0)
        s.events.push_back({now, machine::EventKind::StepCompleted, machine::Severity::Info,
                            "Step done: " + step_label(last_step_)});
      s.events.push_back({now, machine::EventKind::StepStarted, machine::Severity::Info,
                          "Step: " + step_label(cmd)});
      last_step_ = cmd;
    }
    finish_frame(s, cmd, moving);
    apply_fault_fields(s);
    return s;
  }

 private:
  void begin_execution() {
    running_ = true;
    paused_ = false;
    started_clock_ = false;
    last_step_ = -1;
    completed_emitted_ = false;
    frozen_t_ = 0.0;
    // Re-arm the fault scenario and clear any latched fault from a previous run, so
    // each start replays the same deterministic sequence.
    faulted_ = false;
    arc_fault_ = false;
    fault_severity_ = machine::Severity::Info;
    fault_code_ = 0;
    fault_message_.clear();
    prev_elapsed_ = 0.0;
    prev_fault_step_ = -1;
    faults_.arm();
    state_ = machine::ProgramState::Running;
  }

  [[nodiscard]] std::vector<double> home_pose() const {
    const std::vector<double>& first = plan_.trajectory.first_knot();
    return first.empty() ? std::vector<double>(dof(), 0.0) : first;
  }

  // Weld flag active at the trajectory segment covering time t.
  [[nodiscard]] bool weld_at(double t) const {
    const int seg = plan_.trajectory.segment_at(t);
    if (seg < 0 || seg >= static_cast<int>(plan_.segments.size())) return false;
    return plan_.segments[static_cast<std::size_t>(seg)].weld;
  }

  [[nodiscard]] int command_of(double t) const {
    const int cmd = plan_.trajectory.command_at(t);
    return cmd < 0 ? 0 : cmd;
  }

  [[nodiscard]] std::string step_label(int i) const {
    if (i < 0 || i >= static_cast<int>(task_.size())) return "idle";
    const auto& c = task_[static_cast<std::size_t>(i)];
    return c.label.empty() ? machine::to_string(c.kind) : c.label;
  }

  // Latch a fault raised this tick and mutate the frame to reflect it. A hard fault
  // (E-stop / servo) aborts the run; a soft fault (arc loss) only flags the process.
  void apply_trip(const fault::FaultTrip& trip, sdk::RobotState& s, core::Timestamp now) {
    switch (trip.kind) {
      case fault::FaultKind::EmergencyStop:
        faulted_ = true;
        running_ = false;
        paused_ = false;
        state_ = machine::ProgramState::Aborted;
        fault_severity_ = machine::Severity::Critical;
        fault_code_ = 911;
        fault_message_ = trip.message;
        s.events.push_back({now, machine::EventKind::EmergencyStop, machine::Severity::Critical, trip.message});
        break;
      case fault::FaultKind::ServoFault:
        faulted_ = true;
        running_ = false;
        paused_ = false;
        state_ = machine::ProgramState::Aborted;
        fault_severity_ = machine::Severity::Error;
        fault_code_ = 500;
        fault_message_ = trip.message;
        s.events.push_back({now, machine::EventKind::Error, machine::Severity::Error, trip.message});
        break;
      case fault::FaultKind::ArcLoss:
        arc_fault_ = true;
        s.events.push_back({now, machine::EventKind::Warning, machine::Severity::Warning, trip.message});
        break;
      case fault::FaultKind::EncoderNoise:
      case fault::FaultKind::CalibrationDrift:
        break;  // continuous effects, applied in finish_frame
    }
  }

  // Stamp the latched fault (if any) onto the outgoing frame, and set servo power.
  // Called on every return path so the alarm persists until it is cleared.
  void apply_fault_fields(sdk::RobotState& s) const {
    if (faulted_) {
      s.servo_state = machine::ServoState::Error;
      s.program_state = machine::ProgramState::Aborted;
      s.error_severity = fault_severity_;
      s.error_code = fault_code_;
      s.error_message = fault_message_;
    } else {
      s.servo_state = servo_on_ ? machine::ServoState::On : machine::ServoState::Off;
    }
  }

  void finish_frame(sdk::RobotState& s, int cmd, double moving) const {
    s.current_step = cmd;
    s.current_step_label = step_label(cmd);
    s.speed_fraction = moving;

    // Encoder noise perturbs the *reported* joints (the true trajectory is unchanged);
    // Cartesian telemetry is derived from the noisy joints so the frame stays coherent.
    faults_.perturb_joints(s.joint_positions);

    // Calibration drift slowly shifts the TCP the controller believes it has.
    core::Pose3D tool = tools_.current_offset();
    const core::Vec3 drift = faults_.tcp_drift(frozen_t_);
    tool.position_m.x_m += drift.x_m;
    tool.position_m.y_m += drift.y_m;
    tool.position_m.z_m += drift.z_m;

    const auto fk = machine::forward_kinematics(profile_.axes, s.joint_positions, tool);
    s.tcp_pose = fk.tcp;

    // The weld process drives its own signals while welding; other channels (and
    // every channel on a non-welding robot) hold whatever was last written. A lost
    // arc (arc_fault_) forces the weld signals down even mid-seam.
    const bool weld = running_ && !arc_fault_ && weld_at(frozen_t_);
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

    last_joints_ = s.joint_positions;  // so the next jog starts from the current pose
  }

  machine::MachineProfile profile_;
  motion::MotionLimits limits_;
  machine::MotionTask task_;
  motion::TaskPlan plan_;
  machine::ToolTable tools_;
  machine::ProgramState state_{machine::ProgramState::Idle};

  mutable std::vector<double> last_joints_;
  mutable std::map<std::string, double> io_;
  // mutable: continuous effects (encoder noise) advance the RNG from const finish_frame,
  // like io_ and last_joints_ which the same frame also updates.
  mutable fault::FaultInjector faults_;

  bool servo_on_{false};
  bool running_{false};
  bool paused_{false};
  bool started_clock_{false};
  bool completed_emitted_{false};
  int last_step_{-1};
  std::int64_t start_ns_{0};
  std::int64_t last_ns_{0};
  double frozen_t_{0.0};

  // Fault state: a latched hard fault (E-stop / servo) plus a soft arc-loss flag,
  // and the per-tick tracking the injector's triggers need.
  bool faulted_{false};
  bool arc_fault_{false};
  machine::Severity fault_severity_{machine::Severity::Info};
  int fault_code_{0};
  std::string fault_message_;
  double prev_elapsed_{0.0};
  int prev_fault_step_{-1};
};

}  // namespace cavr::sim
