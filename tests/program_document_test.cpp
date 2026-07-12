#include "ProgramDocument.hpp"

#include <cavr/machine/motion.hpp>

#include <cstdlib>
#include <iostream>
#include <string>

namespace {

void require(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << message << '\n';
    std::exit(1);
  }
}

cavr::machine::MotionCommand command(cavr::machine::MotionKind kind, std::string label) {
  cavr::machine::MotionCommand cmd;
  cmd.kind = kind;
  cmd.label = std::move(label);
  return cmd;
}

}  // namespace

int main() {
  ProgramDocument doc;

  require(doc.empty(), "new document starts empty");
  require(doc.selected_step() == -1, "new document has no selected step");

  doc.append(command(cavr::machine::MotionKind::MoveJ, "home"));
  doc.append(command(cavr::machine::MotionKind::MoveL, "approach"));
  doc.append(command(cavr::machine::MotionKind::Wait, "settle"));

  require(doc.size() == 3, "append adds commands");
  require(doc.selected_step() == 2, "append selects the new step");

  doc.select_step(1);
  require(doc.selected_step() == 1, "select_step accepts a valid index");
  doc.select_step(99);
  require(doc.selected_step() == -1, "select_step clears invalid selection");

  doc.select_step(2);
  require(doc.move_step(2, -1), "move_step moves within bounds");
  require(doc.selected_step() == 1, "move_step keeps selection on the moved command");
  require(doc.task()[1].label == "settle", "move_step swaps command order");
  require(!doc.move_step(0, -1), "move_step rejects underflow");

  require(doc.remove_step(1), "remove_step removes a valid index");
  require(doc.size() == 2, "remove_step shrinks the task");
  require(doc.selected_step() == 1, "remove_step clamps selection");
  require(!doc.remove_step(5), "remove_step rejects out of range");

  cavr::core::Pose3D via;
  via.position_m.x_m = 0.1;
  via.position_m.y_m = 0.2;
  via.position_m.z_m = 0.3;
  doc.set_pending_via(via);
  require(doc.has_pending_via(), "set_pending_via stores a via point");
  const auto first_take = doc.take_pending_via();
  require(first_take.has_value(), "take_pending_via returns the stored via");
  require(!doc.has_pending_via(), "take_pending_via clears the pending via");
  require(!doc.take_pending_via().has_value(), "pending via is single-use");

  const auto timeline = doc.to_timeline();
  require(timeline.steps.size() == 1, "to_timeline creates one operation step");
  require(timeline.steps.front().motion.size() == doc.size(), "to_timeline preserves the task");

  doc.clear();
  require(doc.empty(), "clear empties the document");
  require(doc.selected_step() == -1, "clear resets selection");

  return 0;
}
