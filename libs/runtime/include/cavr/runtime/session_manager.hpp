#pragma once

// SessionManager orchestrates the production workflow against a ControllerAdapter:
//
//   connect -> use/discover profile -> set plan -> validate -> execute -> tick* -> done
//
// It owns no controller (the adapter is injected) and never synthesizes motion:
// every recorded frame comes from adapter.poll(). The recorded SessionLog can be
// saved and replayed.

#include <cavr/adapter_sdk/controller_adapter.hpp>
#include <cavr/runtime/session.hpp>
#include <cavr/runtime/timeline.hpp>
#include <cavr/validation/trajectory_validator.hpp>

#include <string>
#include <utility>

namespace cavr::runtime {

namespace sdk = cavr::adapter_sdk;
namespace validation = cavr::validation;

class SessionManager final {
 public:
  [[nodiscard]] SessionPhase phase() const noexcept { return phase_; }
  [[nodiscard]] const machine::MachineProfile& profile() const noexcept { return log_.profile; }
  [[nodiscard]] Timeline& plan() noexcept { return plan_; }
  [[nodiscard]] const Timeline& plan() const noexcept { return plan_; }
  [[nodiscard]] const SessionLog& log() const noexcept { return log_; }
  [[nodiscard]] const sdk::RobotState& latest() const noexcept { return latest_; }
  [[nodiscard]] bool running() const noexcept { return phase_ == SessionPhase::Executing; }

  [[nodiscard]] sdk::ConnectResult connect(sdk::ControllerAdapter& adapter,
                                           const sdk::ConnectionInfo& info) {
    adapter_ = &adapter;
    auto result = adapter.connect(info);
    if (result.ok()) phase_ = SessionPhase::Connected;
    return result;
  }

  // Use a profile loaded from disk, or discover one from the controller.
  void use_profile(machine::MachineProfile profile) {
    log_.profile = std::move(profile);
    if (phase_ == SessionPhase::Connected || phase_ == SessionPhase::Disconnected)
      phase_ = SessionPhase::Profiled;
  }
  [[nodiscard]] const machine::MachineProfile& discover_profile() {
    if (adapter_) log_.profile = adapter_->discover_profile();
    phase_ = SessionPhase::Profiled;
    return log_.profile;
  }

  void set_plan(Timeline plan) {
    plan_ = std::move(plan);
    log_.timeline.steps = plan_.steps;  // keep plan in the log; events accrue at runtime
    if (phase_ == SessionPhase::Profiled || phase_ == SessionPhase::Validated)
      phase_ = SessionPhase::Planned;
  }

  [[nodiscard]] validation::ValidationReport validate() {
    auto report = validation::validate_task(log_.profile, plan_.to_motion_task());
    if (report.ok() && phase_ == SessionPhase::Planned) phase_ = SessionPhase::Validated;
    return report;
  }

  // Hand the task to the controller and start it. The controller executes; we
  // only monitor from here on.
  [[nodiscard]] bool execute(std::string session_id) {
    if (!adapter_ || !adapter_->is_connected()) return false;
    if (!adapter_->load_task(plan_.to_motion_task())) return false;
    if (!adapter_->start()) return false;
    log_.session_id = std::move(session_id);
    log_.frames.clear();
    log_.timeline.events.clear();
    started_clock_ = false;
    phase_ = SessionPhase::Executing;
    return true;
  }

  // Poll telemetry for one clock tick and record it.
  void tick(core::Timestamp now) {
    if (!adapter_) return;
    latest_ = adapter_->poll(now);
    if (phase_ != SessionPhase::Executing) return;

    if (!started_clock_) { log_.started = now; started_clock_ = true; }
    log_.ended = now;
    log_.frames.push_back(latest_);
    for (const auto& e : latest_.events) {
      log_.timeline.add_controller_event(e, latest_.current_step);
    }
    if (latest_.program_state == machine::ProgramState::Completed) {
      phase_ = SessionPhase::Completed;
    }
  }

  void stop() {
    if (adapter_) adapter_->stop();
    if (phase_ == SessionPhase::Executing) phase_ = SessionPhase::Completed;
  }

  void enter_replay() { phase_ = SessionPhase::Replay; }

 private:
  sdk::ControllerAdapter* adapter_{nullptr};
  SessionPhase phase_{SessionPhase::Disconnected};
  Timeline plan_;
  SessionLog log_;
  sdk::RobotState latest_;
  bool started_clock_{false};
};

}  // namespace cavr::runtime
