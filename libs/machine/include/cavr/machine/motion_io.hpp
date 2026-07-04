#pragma once

// Canonical JSON serialization for a MotionTask (a robot program / job). Lives in
// the machine layer so both the wire protocol and the program store share one
// shape — a task saved to the database and a task sent over TCP are byte-identical.
// Reuses the pose/vec serializers from profile_io.

#include <cavr/machine/enums.hpp>
#include <cavr/machine/json.hpp>
#include <cavr/machine/motion.hpp>
#include <cavr/machine/profile_io.hpp>  // detail::pose_to_json / pose_from_json

#include <string>
#include <utility>
#include <vector>

namespace cavr::machine {

[[nodiscard]] inline json::Value command_to_json(const MotionCommand& cmd) {
  json::Value j;
  j.set("kind", to_string(cmd.kind));
  j.set("speed", cmd.speed);
  j.set("blend_radius_m", cmd.blend_radius_m);
  j.set("tool_frame", cmd.tool_frame);
  j.set("user_frame", cmd.user_frame);
  j.set("wait_s", cmd.wait_s);
  j.set("label", cmd.label);
  if (cmd.target.joints) {
    json::Array joints;
    for (double q : *cmd.target.joints) joints.push_back(q);
    j.set("joints", std::move(joints));
  }
  if (cmd.target.pose) j.set("pose", detail::pose_to_json(*cmd.target.pose));
  if (cmd.via) j.set("via", detail::pose_to_json(*cmd.via));
  if (cmd.weld) {
    json::Value w;
    w.set("enabled", cmd.weld->enabled);
    w.set("travel_speed_mm_s", cmd.weld->travel_speed_mm_s);
    w.set("segment_length_mm", cmd.weld->segment_length_mm);
    w.set("tolerance_mm", cmd.weld->tolerance_mm);
    w.set("process_program", cmd.weld->process_program);
    j.set("weld", std::move(w));
  }
  return j;
}

[[nodiscard]] inline MotionCommand command_from_json(const json::Value& j) {
  MotionCommand cmd;
  cmd.kind = motion_kind_from_string(j.at("kind").as_string("movej"));
  cmd.speed = j.at("speed").as_number();
  cmd.blend_radius_m = j.at("blend_radius_m").as_number();
  cmd.tool_frame = j.at("tool_frame").as_string();
  cmd.user_frame = j.at("user_frame").as_string();
  cmd.wait_s = j.at("wait_s").as_number();
  cmd.label = j.at("label").as_string();
  if (const json::Value* joints = j.find("joints"); joints && joints->is_array()) {
    std::vector<double> q;
    for (const auto& v : joints->as_array()) q.push_back(v.as_number());
    cmd.target.joints = std::move(q);
  }
  if (const json::Value* pose = j.find("pose"); pose && pose->is_object())
    cmd.target.pose = detail::pose_from_json(*pose);
  if (const json::Value* via = j.find("via"); via && via->is_object())
    cmd.via = detail::pose_from_json(*via);
  if (const json::Value* w = j.find("weld"); w && w->is_object()) {
    WeldPass pass;
    pass.enabled = w->at("enabled").as_bool();
    pass.travel_speed_mm_s = w->at("travel_speed_mm_s").as_number(8.0);
    pass.segment_length_mm = w->at("segment_length_mm").as_number(2.0);
    pass.tolerance_mm = w->at("tolerance_mm").as_number(0.5);
    pass.process_program = w->at("process_program").as_string();
    cmd.weld = pass;
  }
  return cmd;
}

[[nodiscard]] inline json::Value task_to_json(const MotionTask& task) {
  json::Array arr;
  for (const auto& cmd : task) arr.push_back(command_to_json(cmd));
  return json::Value(std::move(arr));
}

[[nodiscard]] inline MotionTask task_from_json(const json::Value& j) {
  MotionTask task;
  if (j.is_array()) {
    for (const auto& c : j.as_array()) task.push_back(command_from_json(c));
  }
  return task;
}

[[nodiscard]] inline std::string export_task_string(const MotionTask& task) {
  return task_to_json(task).dump(2);
}

[[nodiscard]] inline MotionTask parse_task(std::string_view text) {
  std::string error;
  auto value = json::parse(text, error);
  if (!value) return {};
  return task_from_json(*value);
}

}  // namespace cavr::machine
