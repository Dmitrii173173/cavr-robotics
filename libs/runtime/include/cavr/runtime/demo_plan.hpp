#pragma once

// A representative welding workflow used by the Studio demo and tests:
// scan -> calibration -> point-cloud capture -> trajectory planning ->
// validation -> approach -> weld pass -> retract -> home. Motion steps carry
// MoveJ joint waypoints (within the GP25 limits) so the mock controller can
// execute a real, bounded trajectory.

#include <cavr/machine/motion.hpp>
#include <cavr/runtime/timeline.hpp>

#include <cmath>
#include <vector>

namespace cavr::runtime {

namespace {
[[nodiscard]] inline double demo_deg(double d) { return d * 3.14159265358979323846 / 180.0; }
}  // namespace

[[nodiscard]] inline machine::MotionCommand demo_movej(std::vector<double> joints, double speed,
                                                       std::string label, bool weld = false) {
  machine::MotionCommand cmd;
  cmd.kind = machine::MotionKind::MoveJ;
  cmd.target.joints = std::move(joints);
  cmd.speed = speed;
  cmd.tool_frame = "tcp";
  cmd.user_frame = "table";
  cmd.label = std::move(label);
  if (weld) {
    machine::WeldPass pass;
    pass.enabled = true;
    pass.travel_speed_mm_s = 8.0;
    pass.process_program = "WELD_JOB_12";
    cmd.weld = pass;
  }
  return cmd;
}

[[nodiscard]] inline Timeline make_demo_plan() {
  using std::vector;
  const double sp = demo_deg(45);  // rad/s, comfortably under axis maxima

  const vector<double> home{0, 0, 0, 0, 0, 0};
  const vector<double> scan{demo_deg(30), demo_deg(20), demo_deg(-20), 0, demo_deg(40), 0};
  const vector<double> approach{demo_deg(20), demo_deg(30), demo_deg(-30), 0, demo_deg(50), 0};
  const vector<double> weld_a{demo_deg(15), demo_deg(35), demo_deg(-35), demo_deg(10), demo_deg(55), demo_deg(20)};
  const vector<double> weld_b{demo_deg(-10), demo_deg(35), demo_deg(-35), demo_deg(-10), demo_deg(55), demo_deg(-20)};

  Timeline t;
  int id = 0;
  auto add = [&](OperationKind kind, std::string label, double dur, machine::MotionTask motion) {
    OperationStep s;
    s.id = id++;
    s.kind = kind;
    s.label = std::move(label);
    s.planned_duration_s = dur;
    s.motion = std::move(motion);
    t.steps.push_back(std::move(s));
  };

  add(OperationKind::Scan, "scan part", 1.2, {demo_movej(scan, sp, "scan part")});
  add(OperationKind::Calibration, "hand-eye calibration", 0.5, {});
  add(OperationKind::PointCloudCapture, "capture point cloud", 0.8, {});
  add(OperationKind::TrajectoryPlanning, "plan weld trajectory", 0.5, {});
  add(OperationKind::Validation, "validate trajectory", 0.3, {});
  add(OperationKind::RobotMotion, "approach seam", 1.0, {demo_movej(approach, sp, "approach seam")});

  machine::MotionCommand tool_on;
  tool_on.kind = machine::MotionKind::ToolOn;
  tool_on.label = "arc on";
  machine::MotionCommand tool_off;
  tool_off.kind = machine::MotionKind::ToolOff;
  tool_off.label = "arc off";

  add(OperationKind::Welding, "weld seam",
      2.5, {tool_on, demo_movej(weld_a, demo_deg(15), "weld a-b", true),
            demo_movej(weld_b, demo_deg(15), "weld b-c", true), tool_off});
  add(OperationKind::RobotMotion, "retract", 1.0, {demo_movej(approach, sp, "retract")});
  add(OperationKind::RobotMotion, "return home", 1.2, {demo_movej(home, sp, "return home")});
  return t;
}

}  // namespace cavr::runtime
