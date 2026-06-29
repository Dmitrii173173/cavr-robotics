#include <cavr/adapters/mock_robot/mock_controller.hpp>
#include <cavr/machine/profile_io.hpp>
#include <cavr/runtime/demo_plan.hpp>
#include <cavr/runtime/session_io.hpp>
#include <cavr/runtime/session_manager.hpp>
#include <cavr/validation/trajectory_validator.hpp>

#include <cstdio>
#include <filesystem>
#include <iostream>
#include <string_view>

namespace {

int failures = 0;

void check(bool condition, std::string_view message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
    ++failures;
  }
}

namespace machine = cavr::machine;
namespace runtime = cavr::runtime;
namespace validation = cavr::validation;
namespace mock = cavr::adapters::mock_robot;
namespace sdk = cavr::adapter_sdk;

void test_profile_round_trip() {
  const machine::MachineProfile original = mock::make_gp25_profile();
  const std::string text = machine::export_profile_string(original);

  const auto parsed = machine::parse_profile(text);
  check(parsed.ok(), "profile parses without errors");
  const machine::MachineProfile& p = parsed.profile;

  check(p.id == original.id, "profile id round-trips");
  check(p.robot_model == "Yaskawa Motoman GP25", "robot model round-trips");
  check(p.dof() == 6, "profile has six axes");
  check(p.axes.front().name == "S" && p.axes.back().name == "T", "axis names round-trip");
  check(p.frames.size() == original.frames.size(), "frames round-trip");
  check(p.io.size() == original.io.size(), "io round-trips");
  check(p.cameras.size() == 1 && p.cameras.front().provides_point_cloud,
        "camera with point cloud round-trips");
  check(p.weld.enabled && p.weld.process_program == "WELD_JOB_12", "weld defaults round-trip");
  check(std::abs(p.axes[0].upper_limit - original.axes[0].upper_limit) < 1e-6,
        "axis limits round-trip");
}

void test_validation() {
  const machine::MachineProfile profile = mock::make_gp25_profile();
  const machine::MotionTask good = runtime::make_demo_plan().to_motion_task();
  const auto ok_report = validation::validate_task(profile, good);
  check(ok_report.ok(), "demo plan validates clean");
  check(!ok_report.collisions_evaluated, "collision check honestly reported as not evaluated");

  // an out-of-range MoveJ must be rejected
  machine::MotionTask bad;
  machine::MotionCommand cmd;
  cmd.kind = machine::MotionKind::MoveJ;
  cmd.target.joints = std::vector<double>{0, 0, 0, 0, 0, 100.0};  // 100 rad >> T limit
  bad.push_back(cmd);
  const auto bad_report = validation::validate_task(profile, bad);
  check(!bad_report.ok(), "out-of-range target fails validation");
  check(bad_report.error_count() >= 1, "validation reports the error");
}

void test_session_workflow() {
  mock::MockController controller;
  runtime::SessionManager manager;

  const auto connected = manager.connect(controller, {"mock", "mock"});
  check(connected.ok(), "session connects to mock controller");
  check(manager.phase() == runtime::SessionPhase::Connected, "phase is Connected");

  const auto& profile = manager.discover_profile();
  check(profile.dof() == 6, "discovered profile has six axes");
  check(manager.phase() == runtime::SessionPhase::Profiled, "phase is Profiled");

  manager.set_plan(runtime::make_demo_plan());
  check(manager.phase() == runtime::SessionPhase::Planned, "phase is Planned");

  const auto report = manager.validate();
  check(report.ok(), "plan validates inside the session");
  check(manager.phase() == runtime::SessionPhase::Validated, "phase is Validated");

  check(manager.execute("session_test"), "execute starts the controller");
  check(manager.phase() == runtime::SessionPhase::Executing, "phase is Executing");

  // drive a 50 Hz clock until the program completes
  const std::int64_t step_ns = 20'000'000;  // 20 ms
  std::int64_t now_ns = 1'000'000'000;
  int ticks = 0;
  bool saw_step_event = false;
  while (manager.phase() == runtime::SessionPhase::Executing && ticks < 5000) {
    manager.tick(cavr::core::Timestamp::from_nanoseconds(now_ns));
    for (const auto& e : manager.latest().events) {
      if (e.kind == machine::EventKind::StepStarted) saw_step_event = true;
    }
    now_ns += step_ns;
    ++ticks;
  }

  check(manager.phase() == runtime::SessionPhase::Completed, "program reaches Completed");
  check(manager.latest().program_state == machine::ProgramState::Completed, "telemetry reports Completed");
  check(manager.log().frame_count() > 10, "telemetry frames were recorded");
  check(saw_step_event, "step-start events were emitted during execution");
  check(!manager.log().timeline.events.empty(), "timeline accrued controller events");

  // the demo plan returns home -> all joints near zero at the end
  const auto& last = manager.latest();
  check(last.joint_positions.size() == 6, "final telemetry has six joints");
  double max_abs = 0.0;
  for (double q : last.joint_positions) max_abs = std::max(max_abs, std::abs(q));
  check(max_abs < 1e-3, "robot returns to home at end of plan");

  // ---- session log save + reload + replay ----
  const auto tmp = std::filesystem::temp_directory_path() / "cavr_session_test.json";
  std::vector<std::string> errors;
  check(runtime::save_session_log(manager.log(), tmp, errors), "session log saves");
  check(errors.empty(), "no save errors");

  const auto loaded = runtime::load_session_log(tmp);
  check(loaded.ok(), "session log reloads");
  check(loaded.log.frame_count() == manager.log().frame_count(), "frame count survives round-trip");
  check(loaded.log.profile.dof() == 6, "profile survives in the log");

  runtime::ReplayCursor cursor(loaded.log);
  check(cursor.duration_s() > 0.0, "replay has a positive duration");
  const auto* mid = cursor.frame_at(cursor.duration_s() * 0.5);
  check(mid != nullptr && mid->joint_positions.size() == 6, "replay yields a six-joint frame");

  std::error_code ec;
  std::filesystem::remove(tmp, ec);
}

}  // namespace

int main() {
  test_profile_round_trip();
  test_validation();
  test_session_workflow();

  if (failures != 0) {
    std::cerr << failures << " runtime workflow test(s) failed\n";
    return 1;
  }
  std::cout << "runtime workflow tests passed\n";
  return 0;
}
