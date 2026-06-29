#pragma once

// Pre-execution checks on a planned task against a machine profile: joint-limit
// and speed compliance, axis count, and referenced-frame correctness. This is a
// first, honest pass — full collision and singularity analysis is out of scope
// and is reported as "not evaluated" rather than silently passing.

#include <cavr/machine/kinematics.hpp>
#include <cavr/machine/machine_profile.hpp>
#include <cavr/machine/motion.hpp>

#include <algorithm>
#include <cmath>
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

  return report;
}

}  // namespace cavr::validation
