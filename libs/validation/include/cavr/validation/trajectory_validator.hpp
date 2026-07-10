#pragma once

// Pre-execution checks on a planned task against a machine profile: joint-limit
// and speed compliance, axis count, and referenced-frame correctness. This is a
// first, honest pass — full collision and singularity analysis is out of scope
// and is reported as "not evaluated" rather than silently passing.

#include <cavr/machine/kinematics.hpp>
#include <cavr/machine/machine_profile.hpp>
#include <cavr/machine/motion.hpp>
#include <cavr/motion/limits.hpp>
#include <cavr/motion/plan_task.hpp>
#include <cavr/validation/collision.hpp>

#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <vector>

namespace cavr::validation {

namespace machine = cavr::machine;

struct Issue final {
  machine::Severity severity{machine::Severity::Info};
  std::string message;
  int step_index{-1};
};

struct ValidationReport final {
  std::vector<Issue> issues;
  bool collisions_evaluated{false};   // honest: not implemented yet
  double estimated_cycle_time_s{0.0}; // from the same planner the controller runs

  [[nodiscard]] bool ok() const noexcept {
    return std::none_of(issues.begin(), issues.end(), [](const Issue& i) {
      return i.severity == machine::Severity::Error || i.severity == machine::Severity::Critical;
    });
  }
  [[nodiscard]] std::size_t error_count() const noexcept {
    return static_cast<std::size_t>(std::count_if(issues.begin(), issues.end(), [](const Issue& i) {
      return i.severity == machine::Severity::Error || i.severity == machine::Severity::Critical;
    }));
  }
};

[[nodiscard]] inline bool frame_exists(const machine::MachineProfile& p, const std::string& name) {
  return name.empty() || p.frame(name) != nullptr;
}

[[nodiscard]] inline ValidationReport validate_task(const machine::MachineProfile& profile,
                                                    const machine::MotionTask& task) {
  ValidationReport report;
  const std::size_t dof = profile.dof();

  for (std::size_t s = 0; s < task.size(); ++s) {
    const auto& cmd = task[s];
    const int step = static_cast<int>(s);

    if (cmd.kind == machine::MotionKind::MoveJ && cmd.target.joints) {
      const auto& q = *cmd.target.joints;
      if (q.size() != dof) {
        report.issues.push_back({machine::Severity::Error,
                                 "MoveJ has " + std::to_string(q.size()) + " joints, profile has " +
                                     std::to_string(dof),
                                 step});
      }
      for (std::size_t i = 0; i < std::min(q.size(), dof); ++i) {
        const auto& axis = profile.axes[i];
        if (q[i] < axis.lower_limit || q[i] > axis.upper_limit) {
          report.issues.push_back({machine::Severity::Error,
                                   "Axis " + axis.name + " target out of range", step});
        }
        if (cmd.speed > 0 && cmd.speed > axis.max_speed + 1e-9) {
          report.issues.push_back({machine::Severity::Warning,
                                   "Commanded speed exceeds axis " + axis.name + " maximum", step});
        }
      }
    }

    if ((cmd.kind == machine::MotionKind::MoveL || cmd.kind == machine::MotionKind::MoveC) &&
        !cmd.target.pose) {
      report.issues.push_back({machine::Severity::Error, "MoveL/MoveC has no Cartesian target", step});
    }

    if (!frame_exists(profile, cmd.tool_frame)) {
      report.issues.push_back({machine::Severity::Warning, "Unknown tool frame: " + cmd.tool_frame, step});
    }
    if (!frame_exists(profile, cmd.user_frame)) {
      report.issues.push_back({machine::Severity::Warning, "Unknown user frame: " + cmd.user_frame, step});
    }
    if (cmd.weld && cmd.weld->enabled && !profile.weld.enabled) {
      report.issues.push_back({machine::Severity::Warning,
                               "Weld pass requested but profile has welding disabled", step});
    }
  }

  // Reachability + cycle time from the SAME planner the controller executes, so
  // "what was validated" and "what will run" are one code path. Plan from home
  // with the profile's tool frame; a Cartesian target whose IK leaves the
  // workspace is a hard error (the controller could not run it either).
  core::Pose3D tool{};
  if (const machine::CoordinateFrame* tcp = profile.frame("tcp")) tool = tcp->transform;
  const cavr::motion::TaskPlan plan = cavr::motion::plan_task(
      profile.axes, tool, task, std::vector<double>(dof, 0.0), cavr::motion::MotionLimits::trapezoidal());
  report.estimated_cycle_time_s = plan.cycle_time_s();
  if (!plan.reachable) {
    report.issues.push_back({machine::Severity::Error, "Cartesian target is unreachable (IK did not converge)",
                             plan.unreachable_command});
  }

  return report;
}

// Same validation, plus collision checking against a CollisionModel (self, floor,
// sphere obstacles). The planned trajectory — from the same planner the controller
// runs — is sampled and every configuration is checked, so the report covers the
// whole motion, not just the endpoints. Sets collisions_evaluated = true.
[[nodiscard]] inline ValidationReport validate_task(const machine::MachineProfile& profile,
                                                    const machine::MotionTask& task,
                                                    const CollisionModel& model) {
  ValidationReport report = validate_task(profile, task);
  report.collisions_evaluated = true;

  const std::size_t dof = profile.dof();
  core::Pose3D tool{};
  if (const machine::CoordinateFrame* tcp = profile.frame("tcp")) tool = tcp->transform;
  const cavr::motion::TaskPlan plan = cavr::motion::plan_task(
      profile.axes, tool, task, std::vector<double>(dof, 0.0), cavr::motion::MotionLimits::trapezoidal());

  // Worst penetration per colliding pair, and the program step where it peaked, so
  // one grazing pair yields one issue rather than hundreds of samples.
  std::map<std::string, CollisionHit> worst;
  std::map<std::string, int> worst_step;
  auto record = [&](const std::vector<double>& q, int step) {
    for (const CollisionHit& hit : check_configuration(profile.axes, q, tool, model)) {
      const std::string key = hit.a + " | " + hit.b;
      const auto it = worst.find(key);
      if (it == worst.end() || hit.penetration_m > it->second.penetration_m) {
        worst[key] = hit;
        worst_step[key] = step;
      }
    }
  };

  if (plan.trajectory.empty()) {
    record(std::vector<double>(dof, 0.0), -1);  // no motion: at least check the home pose
  } else {
    const double total = plan.trajectory.duration();
    constexpr int kSteps = 200;
    for (int i = 0; i <= kSteps; ++i) {
      const double t = total * i / kSteps;
      record(plan.trajectory.sample(t), plan.trajectory.command_at(t));
    }
  }

  for (const auto& [key, hit] : worst) {
    const int mm = static_cast<int>(std::lround(hit.penetration_m * 1000.0));
    report.issues.push_back({machine::Severity::Error,
                             "Collision: " + hit.a + " vs " + hit.b + " (penetration ~" +
                                 std::to_string(mm) + " mm)",
                             worst_step[key]});
  }

  return report;
}

}  // namespace cavr::validation
