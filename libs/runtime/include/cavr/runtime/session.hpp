#pragma once

// A SessionLog captures everything needed to diagnose or replay an operation:
// the machine profile it ran against, the planned timeline (+ events that fired)
// and the recorded telemetry stream. SessionPhase tracks where the workflow is.

#include <cavr/adapter_sdk/robot_state.hpp>
#include <cavr/core/time.hpp>
#include <cavr/machine/machine_profile.hpp>
#include <cavr/runtime/timeline.hpp>

#include <array>
#include <string>
#include <string_view>
#include <vector>

namespace cavr::runtime {

// Scan -> Plan -> Validate -> Execute -> Monitor -> Replay.
enum class SessionPhase {
  Disconnected, Connected, Profiled, Planned, Validated, Executing, Completed, Replay
};

namespace detail {
inline constexpr std::array<std::pair<SessionPhase, std::string_view>, 8> kSessionPhase{{
    {SessionPhase::Disconnected, "disconnected"}, {SessionPhase::Connected, "connected"},
    {SessionPhase::Profiled, "profiled"}, {SessionPhase::Planned, "planned"},
    {SessionPhase::Validated, "validated"}, {SessionPhase::Executing, "executing"},
    {SessionPhase::Completed, "completed"}, {SessionPhase::Replay, "replay"}}};
}  // namespace detail

[[nodiscard]] inline std::string to_string(SessionPhase v) {
  return machine::detail::enum_to_string(v, detail::kSessionPhase);
}

struct SessionLog final {
  std::string session_id;
  machine::MachineProfile profile;
  Timeline timeline;                       // plan + recorded events
  std::vector<sdk::RobotState> frames;     // recorded telemetry
  core::Timestamp started{core::Timestamp::zero()};
  core::Timestamp ended{core::Timestamp::zero()};

  [[nodiscard]] bool empty() const noexcept { return frames.empty(); }
  [[nodiscard]] std::size_t frame_count() const noexcept { return frames.size(); }
};

// Steps through a recorded SessionLog by wall-clock offset for replay/diagnostics.
class ReplayCursor final {
 public:
  explicit ReplayCursor(const SessionLog& log) : log_(&log) {}

  // Returns the frame active at `seconds` past the session start (clamped).
  [[nodiscard]] const sdk::RobotState* frame_at(double seconds) const {
    if (log_->frames.empty()) return nullptr;
    const std::int64_t base = log_->frames.front().timestamp.nanoseconds();
    const std::int64_t want = base + static_cast<std::int64_t>(seconds * 1e9);
    const sdk::RobotState* chosen = &log_->frames.front();
    for (const auto& f : log_->frames) {
      if (f.timestamp.nanoseconds() <= want) chosen = &f;
      else break;
    }
    return chosen;
  }

  [[nodiscard]] double duration_s() const {
    if (log_->frames.size() < 2) return 0.0;
    return static_cast<double>(log_->frames.back().timestamp.nanoseconds() -
                               log_->frames.front().timestamp.nanoseconds()) *
           1e-9;
  }

 private:
  const SessionLog* log_;
};

}  // namespace cavr::runtime
