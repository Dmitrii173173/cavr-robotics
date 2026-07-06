#pragma once

// Translate an industrial MotionTask into a time-parameterised Trajectory. This is
// the single planner that both execution (VirtualRobot) and validation (cycle
// time / reachability) call, so "what was checked" and "what will run" are the
// same code — the thing that was previously duplicated between the mock's
// rebuild_schedule and the validator.
//
// Geometry (FK/IK/arc) comes from libs/machine; the velocity profile and blending
// come from the rest of libs/motion. A MoveJ is timed in joint space (rad), a
// MoveL/MoveC in Cartesian space (mm) — each segment carries its own path unit, so
// the profile is applied consistently to either.

#include <cavr/machine/arc.hpp>
#include <cavr/machine/ik.hpp>
#include <cavr/machine/kinematics.hpp>
#include <cavr/machine/machine_profile.hpp>
#include <cavr/machine/motion.hpp>
#include <cavr/motion/limits.hpp>
#include <cavr/motion/trajectory.hpp>

#include <cmath>
#include <cstddef>
#include <vector>

namespace cavr::motion {

namespace machine = cavr::machine;

// Per-segment execution metadata, parallel to the trajectory's segments. The
// executor uses it to drive process signals (weld) and to map a segment back to
// its program line.
struct SegmentInfo final {
  int command_index{0};
  bool motion{false};
  bool weld{false};
};

struct TaskPlan final {
  Trajectory trajectory;
  std::vector<SegmentInfo> segments;   // parallel to trajectory segments
  bool reachable{true};                // false if any IK left the workspace
  int unreachable_command{-1};         // first command whose target was unreachable

  [[nodiscard]] double cycle_time_s() const noexcept { return trajectory.duration(); }
};

namespace plan_detail {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDefaultJointSpeed = kPi / 3.0;  // 60 deg/s
constexpr double kDefaultCartSpeed = 50.0;        // mm/s
constexpr int kArcSamples = 13;

[[nodiscard]] inline double cart_dist_mm(const core::Vec3& a, const core::Vec3& b) {
  const double dx = a.x_m - b.x_m, dy = a.y_m - b.y_m, dz = a.z_m - b.z_m;
  return std::sqrt(dx * dx + dy * dy + dz * dz) * 1000.0;
}

}  // namespace plan_detail

// Build a plan for `task`, starting from `start_joints`, using `tool` as the TCP
// offset. Segments that leave the reachable workspace are marked (reachable=false)
// but still produce a best-effort trajectory so the caller can inspect the rest.
[[nodiscard]] inline TaskPlan plan_task(const std::vector<machine::AxisSpec>& axes,
                                        const core::Pose3D& tool, const machine::MotionTask& task,
                                        const std::vector<double>& start_joints,
                                        const MotionLimits& limits) {
  using namespace plan_detail;
  const std::size_t dof = axes.size();

  std::vector<PlanSegment> segs;
  std::vector<SegmentInfo> infos;
  std::vector<double> current = start_joints;
  current.resize(dof, 0.0);
  bool weld_on = false;

  const auto push = [&](PlanSegment seg, int ci, bool motion, bool weld) {
    seg.command_index = ci;
    seg.motion = motion;
    segs.push_back(std::move(seg));
    infos.push_back({ci, motion, weld});
  };

  TaskPlan result;

  for (std::size_t ci = 0; ci < task.size(); ++ci) {
    const auto& cmd = task[ci];
    const int idx = static_cast<int>(ci);
    const bool cmd_weld = weld_on || (cmd.weld && cmd.weld->enabled);
    const double blend_mm = cmd.blend_radius_m > 0.0 ? cmd.blend_radius_m * 1000.0 : 0.0;

    if (cmd.kind == machine::MotionKind::MoveJ && cmd.target.joints) {
      std::vector<double> next = *cmd.target.joints;
      next.resize(dof, 0.0);
      double sweep = 0.0;
      for (std::size_t i = 0; i < dof; ++i) sweep = std::max(sweep, std::abs(next[i] - current[i]));
      PlanSegment seg;
      seg.knots = {current, next};
      seg.knot_s = {0.0, sweep};
      seg.speed = cmd.speed > 0.0 ? cmd.speed : kDefaultJointSpeed;  // rad/s
      push(std::move(seg), idx, true, cmd_weld);                     // MoveJ blend units are rad; left off
      current = std::move(next);
    } else if (cmd.kind == machine::MotionKind::MoveL && cmd.target.pose) {
      const machine::IkResult ik = machine::inverse_kinematics(axes, *cmd.target.pose, current, tool);
      std::vector<double> next = ik.converged ? ik.joints : current;
      next.resize(dof, 0.0);
      if (!ik.converged && result.reachable) { result.reachable = false; result.unreachable_command = idx; }
      const core::Pose3D cur_pose = machine::forward_kinematics(axes, current, tool).tcp;
      const double dist = cart_dist_mm(cmd.target.pose->position_m, cur_pose.position_m);
      PlanSegment seg;
      seg.knots = {current, next};
      seg.knot_s = {0.0, dist};
      seg.speed = cmd.speed > 0.0 ? cmd.speed : kDefaultCartSpeed;  // mm/s
      seg.blend_radius = blend_mm;
      push(std::move(seg), idx, true, cmd_weld);
      current = std::move(next);
    } else if (cmd.kind == machine::MotionKind::MoveC && cmd.target.pose) {
      const core::Pose3D start_pose = machine::forward_kinematics(axes, current, tool).tcp;
      const core::Pose3D via = cmd.via ? *cmd.via : start_pose;
      const machine::ArcPath arc = machine::sample_arc(start_pose, via, *cmd.target.pose, kArcSamples);
      PlanSegment seg;
      seg.knots.reserve(arc.poses.size());
      seg.knot_s.reserve(arc.poses.size());
      seg.knots.push_back(current);
      seg.knot_s.push_back(0.0);
      std::vector<double> seed = current;
      double cum = 0.0;
      for (std::size_t i = 1; i < arc.poses.size(); ++i) {
        const machine::IkResult ik = machine::inverse_kinematics(axes, arc.poses[i], seed, tool);
        std::vector<double> next = ik.converged ? ik.joints : seed;
        next.resize(dof, 0.0);
        if (!ik.converged && result.reachable) { result.reachable = false; result.unreachable_command = idx; }
        cum += cart_dist_mm(arc.poses[i].position_m, arc.poses[i - 1].position_m);
        seg.knots.push_back(next);
        seg.knot_s.push_back(cum);
        seed = std::move(next);
      }
      seg.speed = cmd.speed > 0.0 ? cmd.speed : kDefaultCartSpeed;  // mm/s
      seg.blend_radius = blend_mm;
      push(std::move(seg), idx, true, cmd_weld);
      current = std::move(seed);
    } else if (cmd.kind == machine::MotionKind::Wait) {
      PlanSegment seg;
      seg.knots = {current};
      seg.knot_s = {0.0};
      seg.hold_s = std::max(0.0, cmd.wait_s);
      seg.motion = false;
      push(std::move(seg), idx, false, cmd_weld);
    } else if (cmd.kind == machine::MotionKind::ToolOn || cmd.kind == machine::MotionKind::ToolOff) {
      weld_on = (cmd.kind == machine::MotionKind::ToolOn);
      PlanSegment seg;
      seg.knots = {current};
      seg.knot_s = {0.0};
      seg.hold_s = 0.15;
      seg.motion = false;
      push(std::move(seg), idx, false, weld_on);
    } else {
      PlanSegment seg;
      seg.knots = {current};
      seg.knot_s = {0.0};
      seg.hold_s = 0.1;
      seg.motion = false;
      push(std::move(seg), idx, false, cmd_weld);
    }
  }

  result.trajectory = plan(segs, limits);
  result.segments = std::move(infos);
  return result;
}

}  // namespace cavr::motion
