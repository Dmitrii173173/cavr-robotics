#include "ProgramDocument.hpp"

#include <utility>

void ProgramDocument::set_task(cavr::machine::MotionTask task) {
  task_ = std::move(task);
  pending_via_.reset();
  selected_step_ = task_.empty() ? -1 : 0;
}

void ProgramDocument::clear() {
  task_.clear();
  pending_via_.reset();
  selected_step_ = -1;
}

void ProgramDocument::select_step(int index) {
  if (index < 0 || index >= static_cast<int>(task_.size())) {
    selected_step_ = -1;
    return;
  }
  selected_step_ = index;
}

bool ProgramDocument::remove_step(int index) {
  if (index < 0 || index >= static_cast<int>(task_.size())) return false;
  task_.erase(task_.begin() + index);
  clamp_selection();
  return true;
}

bool ProgramDocument::move_step(int index, int delta) {
  const int target = index + delta;
  if (index < 0 || index >= static_cast<int>(task_.size())) return false;
  if (target < 0 || target >= static_cast<int>(task_.size())) return false;
  std::swap(task_[static_cast<std::size_t>(index)], task_[static_cast<std::size_t>(target)]);
  if (selected_step_ == index) {
    selected_step_ = target;
  } else if (selected_step_ == target) {
    selected_step_ = index;
  }
  return true;
}

void ProgramDocument::append(cavr::machine::MotionCommand command) {
  task_.push_back(std::move(command));
  select_last();
}

void ProgramDocument::set_pending_via(cavr::core::Pose3D pose) {
  pending_via_ = pose;
}

std::optional<cavr::core::Pose3D> ProgramDocument::take_pending_via() {
  auto via = pending_via_;
  pending_via_.reset();
  return via;
}

cavr::runtime::Timeline ProgramDocument::to_timeline() const {
  cavr::runtime::Timeline plan;
  if (task_.empty()) return plan;

  cavr::runtime::OperationStep step;
  step.id = 0;
  step.kind = cavr::runtime::OperationKind::RobotMotion;
  step.label = "program";
  step.motion = task_;
  plan.steps.push_back(std::move(step));
  return plan;
}

void ProgramDocument::select_last() {
  selected_step_ = task_.empty() ? -1 : static_cast<int>(task_.size()) - 1;
}

void ProgramDocument::clamp_selection() {
  if (task_.empty()) {
    selected_step_ = -1;
  } else if (selected_step_ >= static_cast<int>(task_.size())) {
    selected_step_ = static_cast<int>(task_.size()) - 1;
  }
}
