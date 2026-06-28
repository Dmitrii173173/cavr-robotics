#pragma once

#include <cavr/replay/session.hpp>

#include <algorithm>
#include <optional>
#include <span>
#include <vector>

namespace cavr::replay {

enum class ReplayMode {
  deterministic,
};

struct ReplayEvent final {
  core::Timestamp timestamp;
  std::size_t sequence_index{};
  ReplaySample sample;
};

class ReplayTimeline final {
 public:
  ReplayTimeline() = default;

  explicit ReplayTimeline(std::vector<ReplayEvent> events) : events_(std::move(events)) {
    std::sort(events_.begin(), events_.end(), [](const ReplayEvent& lhs, const ReplayEvent& rhs) {
      if (lhs.timestamp == rhs.timestamp) {
        return lhs.sequence_index < rhs.sequence_index;
      }
      return lhs.timestamp < rhs.timestamp;
    });
  }

  [[nodiscard]] bool empty() const noexcept {
    return events_.empty();
  }

  [[nodiscard]] std::size_t size() const noexcept {
    return events_.size();
  }

  [[nodiscard]] std::span<const ReplayEvent> events() const noexcept {
    return events_;
  }

  [[nodiscard]] const ReplayEvent* event_at(std::size_t index) const noexcept {
    return index < events_.size() ? &events_[index] : nullptr;
  }

 private:
  std::vector<ReplayEvent> events_;
};

[[nodiscard]] inline ReplayTimeline build_timeline(const LoadedSession& session) {
  std::vector<ReplayEvent> events;
  events.reserve(session.replay_samples.size());
  for (std::size_t index = 0; index < session.replay_samples.size(); ++index) {
    events.push_back(ReplayEvent{
        session.replay_samples[index].timestamp,
        index,
        session.replay_samples[index],
    });
  }
  return ReplayTimeline(std::move(events));
}

class ReplayCursor final {
 public:
  explicit ReplayCursor(const ReplayTimeline& timeline) noexcept : timeline_(&timeline) {}

  [[nodiscard]] std::size_t index() const noexcept {
    return index_;
  }

  [[nodiscard]] bool done() const noexcept {
    return timeline_ == nullptr || index_ >= timeline_->size();
  }

  void reset() noexcept {
    index_ = 0;
  }

  [[nodiscard]] const ReplayEvent* current() const noexcept {
    return timeline_ == nullptr ? nullptr : timeline_->event_at(index_);
  }

  [[nodiscard]] const ReplayEvent* step_next() noexcept {
    if (timeline_ == nullptr) {
      return nullptr;
    }
    const ReplayEvent* event = timeline_->event_at(index_);
    if (event != nullptr) {
      ++index_;
    }
    return event;
  }

  [[nodiscard]] const ReplayEvent* seek_to(core::Timestamp timestamp) noexcept {
    if (timeline_ == nullptr) {
      return nullptr;
    }

    const auto all_events = timeline_->events();
    const auto it = std::lower_bound(
        all_events.begin(), all_events.end(), timestamp,
        [](const ReplayEvent& event, core::Timestamp target) { return event.timestamp < target; });

    index_ = static_cast<std::size_t>(std::distance(all_events.begin(), it));
    return current();
  }

 private:
  const ReplayTimeline* timeline_{};
  std::size_t index_{};
};

}  // namespace cavr::replay
