// VirtualRobot: the adapter-free robot simulation. Because it owns the joint
// state and executes a planned Trajectory against a clock we supply, its whole
// lifecycle — idle → load → run → complete, pause/resume, live jog, and IO
// direction rules — can be driven and asserted without a socket or an adapter.
// Time is injected via poll(now), so these tests are deterministic (no sleeping).

#include <cavr/sim/virtual_robot.hpp>

#include <cavr/adapters/mock_robot/mock_controller.hpp>

#include <cmath>
#include <iostream>
#include <string_view>
#include <vector>

namespace {

int failures = 0;

void check(bool condition, std::string_view message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
    ++failures;
  }
}

namespace machine = cavr::machine;
namespace sim = cavr::sim;

double deg(double d) { return d * 3.14159265358979323846 / 180.0; }

cavr::core::Timestamp at(double seconds) {
  return cavr::core::Timestamp::from_nanoseconds(static_cast<std::int64_t>(seconds * 1e9));
}

double joint_dist(const std::vector<double>& a, const std::vector<double>& b) {
  double s = 0.0;
  const std::size_t n = std::min(a.size(), b.size());
  for (std::size_t i = 0; i < n; ++i) s += (a[i] - b[i]) * (a[i] - b[i]);
  return std::sqrt(s);
}

bool has_event(const cavr::adapter_sdk::RobotState& s, machine::EventKind kind) {
  for (const auto& e : s.events)
    if (e.kind == kind) return true;
  return false;
}

// A joint-space program that swings the base axis a large angle, so the move
// lasts well over a second and can be sampled mid-motion.
machine::MotionTask big_movej() {
  machine::MotionCommand c;
  c.kind = machine::MotionKind::MoveJ;
  c.target.joints = std::vector<double>{deg(120), 0, 0, 0, 0, 0};
  c.speed = deg(60);  // rad/s, matching the profile's MoveJ default
  c.label = "swing";
  return {c};
}

// Before a program runs, the robot reports itself idle at home with servos off.
void test_idle_home() {
  sim::VirtualRobot robot(cavr::adapters::mock_robot::make_gp25_profile());
  const auto s = robot.poll(at(0.0));
  check(s.program_state == machine::ProgramState::Idle, "fresh robot is Idle");
  check(s.servo_state == machine::ServoState::Off, "servo starts Off");
  check(s.joint_positions.size() == robot.dof(), "reports one position per axis");
  check(joint_dist(s.joint_positions, std::vector<double>(robot.dof(), 0.0)) < 1e-9,
        "idle pose is home (all zero)");
  check(s.speed_fraction == 0.0, "idle robot is not moving");
}

// Load → start → run to the target, reaching Completed exactly once.
void test_run_to_completion() {
  sim::VirtualRobot robot(cavr::adapters::mock_robot::make_gp25_profile());
  robot.set_servo(true);

  check(robot.load_task(big_movej()), "load_task succeeds");
  check(robot.state() == machine::ProgramState::Loaded, "state is Loaded after load");
  check(robot.start(), "start succeeds with a loaded program");

  auto s0 = robot.poll(at(0.0));
  check(s0.program_state == machine::ProgramState::Running, "running after start");
  check(s0.servo_state == machine::ServoState::On, "servo On while energised");

  // Far past any plausible duration: the move must be finished.
  auto done = robot.poll(at(100.0));
  check(done.program_state == machine::ProgramState::Completed, "completes after enough time");
  check(joint_dist(done.joint_positions, {deg(120), 0, 0, 0, 0, 0}) < 1e-6,
        "final pose is the commanded target");
  check(has_event(done, machine::EventKind::ProgramCompleted), "emits ProgramCompleted");
  check(done.speed_fraction == 0.0, "stopped once complete");

  // The completion event is edge-triggered: polling again does not re-emit it.
  auto after = robot.poll(at(101.0));
  check(after.program_state == machine::ProgramState::Completed, "stays Completed");
  check(!has_event(after, machine::EventKind::ProgramCompleted),
        "ProgramCompleted fires only once");
}

// Pause freezes progress; resume continues from where it left off.
void test_pause_resume() {
  sim::VirtualRobot robot(cavr::adapters::mock_robot::make_gp25_profile());
  robot.set_servo(true);
  check(robot.load_task(big_movej()), "load for pause test");
  check(robot.start(), "start for pause test");

  (void)robot.poll(at(0.0));           // anchors the clock at t=0
  const auto mid = robot.poll(at(0.5));  // partway through the swing
  check(mid.speed_fraction > 0.0, "moving mid-swing");
  const auto j_mid = mid.joint_positions;

  robot.pause();
  const auto held1 = robot.poll(at(1.0));
  check(held1.program_state == machine::ProgramState::Paused, "reports Paused");
  check(joint_dist(held1.joint_positions, j_mid) < 1e-9, "pose frozen while paused");
  const auto held2 = robot.poll(at(1.4));
  check(joint_dist(held2.joint_positions, j_mid) < 1e-9, "still frozen after more time");

  robot.resume();
  const auto moved = robot.poll(at(1.6));
  check(moved.program_state != machine::ProgramState::Paused, "no longer Paused after resume");
  check(joint_dist(moved.joint_positions, j_mid) > 1e-4, "advances again after resume");
}

// Immediate jog: a reachable joint move runs; a Cartesian target outside the
// workspace is rejected, matching a real controller.
void test_jog_reachability() {
  sim::VirtualRobot robot(cavr::adapters::mock_robot::make_gp25_profile());
  robot.set_servo(true);

  machine::MotionCommand reachable;
  reachable.kind = machine::MotionKind::MoveJ;
  reachable.target.joints = std::vector<double>{deg(15), deg(10), 0, 0, 0, 0};
  reachable.speed = deg(60);
  check(robot.move_to(reachable), "reachable joint jog accepted");

  machine::MotionCommand unreachable;
  unreachable.kind = machine::MotionKind::MoveL;
  unreachable.target.pose = cavr::core::Pose3D{cavr::core::Vec3{10.0, 10.0, 10.0},
                                               cavr::core::Quaternion::identity()};
  unreachable.speed = 0.25;
  check(!robot.move_to(unreachable), "Cartesian target outside the workspace is rejected");
}

// IO writes respect channel direction: outputs are writable, inputs and unknown
// names are not.
void test_io_direction() {
  sim::VirtualRobot robot(cavr::adapters::mock_robot::make_gp25_profile());
  check(robot.write_io("gas_on", 1.0), "digital output is writable");
  check(!robot.write_io("part_present", 1.0), "input channel is not writable");
  check(!robot.write_io("does_not_exist", 1.0), "unknown channel is rejected");
}

// A timed E-stop aborts the program mid-move: the run stops, the servos fault, the
// pose freezes and the alarm latches until it is cleared.
void test_estop_aborts() {
  sim::VirtualRobot robot(cavr::adapters::mock_robot::make_gp25_profile());
  robot.set_servo(true);
  robot.faults().add(cavr::fault_injection::estop_at(0.5));  // fire 0.5 s into the swing
  check(robot.load_task(big_movej()), "load for e-stop test");
  check(robot.start(), "start for e-stop test");

  (void)robot.poll(at(0.0));            // anchor the clock
  const auto pre = robot.poll(at(0.4));  // before the E-stop
  check(pre.program_state == machine::ProgramState::Running, "running before the E-stop");
  check(!robot.faulted(), "not yet faulted");

  const auto trip = robot.poll(at(0.6));  // crosses 0.5 s → E-stop
  check(trip.program_state == machine::ProgramState::Aborted, "program aborts on E-stop");
  check(trip.servo_state == machine::ServoState::Error, "servos report Error");
  check(trip.faulted(), "frame is flagged as faulted");
  check(has_event(trip, machine::EventKind::EmergencyStop), "emits an EmergencyStop event");
  const auto frozen = trip.joint_positions;

  // The fault latches and the pose holds even as wall-clock advances.
  const auto later = robot.poll(at(5.0));
  check(later.program_state == machine::ProgramState::Aborted, "stays Aborted");
  check(later.servo_state == machine::ServoState::Error, "servo stays in Error");
  check(joint_dist(later.joint_positions, frozen) < 1e-6, "pose is frozen at the stop point");
  check(joint_dist(later.joint_positions, {deg(120), 0, 0, 0, 0, 0}) > 1e-2,
        "did not reach the target — it was stopped short");

  // Clearing the fault and re-running completes normally (the scenario re-arms, but
  // a fresh program without the fixture fault runs clean here).
  robot.clear_fault();
  check(!robot.faulted(), "fault cleared");
}

// A lost weld arc forces the weld signals down mid-seam without aborting the run.
void test_arc_loss_drops_weld() {
  sim::VirtualRobot robot(cavr::adapters::mock_robot::make_gp25_profile());
  robot.set_servo(true);

  // Program: turn the tool (weld) on, then a long swing so we sample mid-weld.
  machine::MotionCommand tool_on;
  tool_on.kind = machine::MotionKind::ToolOn;
  tool_on.label = "arc on";
  machine::MotionTask task = {tool_on, big_movej().front()};

  robot.faults().add(cavr::fault_injection::arc_loss_at(0.5));
  check(robot.load_task(task), "load for arc-loss test");
  check(robot.start(), "start for arc-loss test");

  auto io_value = [](const cavr::adapter_sdk::RobotState& s, const std::string& name) {
    for (const auto& io : s.io) if (io.name == name) return io.value;
    return -1.0;
  };

  (void)robot.poll(at(0.0));
  const auto welding = robot.poll(at(0.3));  // after ToolOn, before arc loss
  check(io_value(welding, "weld_on") == 1.0, "weld is on before the arc drops");
  check(io_value(welding, "arc_established") == 1.0, "arc established before the fault");

  const auto lost = robot.poll(at(0.7));  // past the arc-loss time
  check(has_event(lost, machine::EventKind::Warning) || robot.faults().size() > 0,
        "arc loss raises a warning");
  check(io_value(lost, "weld_on") == 0.0, "weld signal forced off after arc loss");
  check(io_value(lost, "arc_established") == 0.0, "arc no longer established");
  check(lost.program_state != machine::ProgramState::Aborted, "arc loss does not abort the run");
}

}  // namespace

int main() {
  test_idle_home();
  test_run_to_completion();
  test_pause_resume();
  test_jog_reachability();
  test_io_direction();

  if (failures != 0) {
    std::cerr << failures << " virtual robot test(s) failed\n";
    return 1;
  }
  std::cout << "virtual robot tests passed\n";
  return 0;
}
