#pragma once

#include <cavr/core/geometry.hpp>
#include <cavr/machine/motion.hpp>
#include <cavr/runtime/timeline.hpp>

#include <cstddef>
#include <optional>

class ProgramDocument final {
 public:
  [[nodiscard]] bool empty() const noexcept { return task_.empty(); }
  [[nodiscard]] std::size_t size() const noexcept { return task_.size(); }
  [[nodiscard]] int selected_step() const noexcept { return selected_step_; }
  [[nodiscard]] const cavr::machine::MotionTask& task() const noexcept { return task_; }

  void set_task(cavr::machine::MotionTask task);
  void clear();

  void select_step(int index);
  [[nodiscard]] bool remove_step(int index);
  [[nodiscard]] bool move_step(int index, int delta);

  void append(cavr::machine::MotionCommand command);
  void set_pending_via(cavr::core::Pose3D pose);
  [[nodiscard]] bool has_pending_via() const noexcept { return pending_via_.has_value(); }
  [[nodiscard]] std::optional<cavr::core::Pose3D> take_pending_via();

  [[nodiscard]] cavr::runtime::Timeline to_timeline() const;

 private:
  void select_last();
  void clamp_selection();

  cavr::machine::MotionTask task_;
  std::optional<cavr::core::Pose3D> pending_via_;
  int selected_step_{-1};
};
