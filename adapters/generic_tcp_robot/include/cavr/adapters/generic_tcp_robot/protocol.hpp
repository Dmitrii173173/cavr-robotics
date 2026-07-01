#pragma once

// The CAVR generic TCP robot wire protocol: newline-delimited JSON, one object
// per line. The adapter is the client; a controller bridge/PLC (or the test
// server) is the server. Kept in one place so both ends share the vocabulary and
// a real bridge can be written against it.
//
// Client -> server (commands):
//   {"cmd":"discover_profile"}
//   {"cmd":"load_task","task":[<command>, ...]}
//   {"cmd":"start"} | {"cmd":"pause"} | {"cmd":"resume"} | {"cmd":"stop"}
//
// Server -> client (replies and telemetry):
//   {"type":"profile","profile":{<MachineProfile>}}   (reply to discover_profile)
//   {"type":"ack","cmd":"start","ok":true}            (reply to a command)
//   {"type":"state", <RobotState fields>}             (unsolicited telemetry stream)
//
// Payloads reuse the project's hand-rolled JSON and the existing profile/pose
// serializers, so a task and a telemetry frame have one canonical shape.

#include <cavr/adapter_sdk/robot_state.hpp>
#include <cavr/machine/enums.hpp>
#include <cavr/machine/json.hpp>
#include <cavr/machine/motion.hpp>
#include <cavr/machine/profile_io.hpp>

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace cavr::adapters::generic_tcp_robot::protocol {

namespace machine = cavr::machine;
namespace sdk = cavr::adapter_sdk;

// -------------------------------------------------------------------- motion task

[[nodiscard]] inline json::Value command_to_json(const machine::MotionCommand& cmd) {
  json::Value j;
  j.set("kind", machine::to_string(cmd.kind));
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
  if (cmd.target.pose) j.set("pose", machine::detail::pose_to_json(*cmd.target.pose));
  if (cmd.via) j.set("via", machine::detail::pose_to_json(*cmd.via));
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

[[nodiscard]] inline machine::MotionCommand command_from_json(const json::Value& j) {
  machine::MotionCommand cmd;
  cmd.kind = machine::motion_kind_from_string(j.at("kind").as_string("movej"));
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
    cmd.target.pose = machine::detail::pose_from_json(*pose);
  if (const json::Value* via = j.find("via"); via && via->is_object())
    cmd.via = machine::detail::pose_from_json(*via);
  if (const json::Value* w = j.find("weld"); w && w->is_object()) {
    machine::WeldPass pass;
    pass.enabled = w->at("enabled").as_bool();
    pass.travel_speed_mm_s = w->at("travel_speed_mm_s").as_number(8.0);
    pass.segment_length_mm = w->at("segment_length_mm").as_number(2.0);
    pass.tolerance_mm = w->at("tolerance_mm").as_number(0.5);
    pass.process_program = w->at("process_program").as_string();
    cmd.weld = pass;
  }
  return cmd;
}

[[nodiscard]] inline json::Value task_to_json(const machine::MotionTask& task) {
  json::Array arr;
  for (const auto& cmd : task) arr.push_back(command_to_json(cmd));
  return json::Value(std::move(arr));
}

[[nodiscard]] inline machine::MotionTask task_from_json(const json::Value& j) {
  machine::MotionTask task;
  if (j.is_array()) {
    for (const auto& c : j.as_array()) task.push_back(command_from_json(c));
  }
  return task;
}

// -------------------------------------------------------------------- robot state

[[nodiscard]] inline json::Value state_to_json(const sdk::RobotState& s) {
  json::Value j;
  j.set("t_ns", static_cast<std::int64_t>(s.timestamp.nanoseconds()));
  json::Array joints;
  for (double q : s.joint_positions) joints.push_back(q);
  j.set("joints", std::move(joints));
  j.set("tcp", machine::detail::pose_to_json(s.tcp_pose));
  j.set("program_state", machine::to_string(s.program_state));
  j.set("servo_state", machine::to_string(s.servo_state));
  j.set("step", s.current_step);
  j.set("step_label", s.current_step_label);
  j.set("speed_fraction", s.speed_fraction);
  j.set("tcp_speed_mm_s", s.tcp_speed_mm_s);
  json::Array io;
  for (const auto& sample : s.io) {
    json::Value e;
    e.set("name", sample.name);
    e.set("value", sample.value);
    io.push_back(std::move(e));
  }
  j.set("io", std::move(io));
  j.set("error_severity", machine::to_string(s.error_severity));
  j.set("error_code", s.error_code);
  j.set("error_message", s.error_message);
  j.set("has_camera_frame", s.has_camera_frame);
  j.set("camera_frame_id", s.camera_frame_id);
  j.set("has_point_cloud", s.has_point_cloud);
  j.set("point_cloud_id", s.point_cloud_id);
  json::Array events;
  for (const auto& e : s.events) {
    json::Value ev;
    ev.set("t_ns", static_cast<std::int64_t>(e.timestamp.nanoseconds()));
    ev.set("kind", machine::to_string(e.kind));
    ev.set("severity", machine::to_string(e.severity));
    ev.set("message", e.message);
    events.push_back(std::move(ev));
  }
  j.set("events", std::move(events));
  return j;
}

[[nodiscard]] inline sdk::RobotState state_from_json(const json::Value& j) {
  sdk::RobotState s;
  s.timestamp = core::Timestamp::from_nanoseconds(j.at("t_ns").as_int());
  if (const json::Value* joints = j.find("joints"); joints && joints->is_array()) {
    for (const auto& v : joints->as_array()) s.joint_positions.push_back(v.as_number());
  }
  if (const json::Value* tcp = j.find("tcp"); tcp && tcp->is_object())
    s.tcp_pose = machine::detail::pose_from_json(*tcp);
  s.program_state = machine::program_state_from_string(j.at("program_state").as_string("idle"));
  s.servo_state = machine::servo_state_from_string(j.at("servo_state").as_string("off"));
  s.current_step = static_cast<int>(j.at("step").as_int(-1));
  s.current_step_label = j.at("step_label").as_string();
  s.speed_fraction = j.at("speed_fraction").as_number();
  s.tcp_speed_mm_s = j.at("tcp_speed_mm_s").as_number();
  if (const json::Value* io = j.find("io"); io && io->is_array()) {
    for (const auto& e : io->as_array())
      s.io.push_back({e.at("name").as_string(), e.at("value").as_number()});
  }
  s.error_severity = machine::severity_from_string(j.at("error_severity").as_string("info"));
  s.error_code = static_cast<int>(j.at("error_code").as_int());
  s.error_message = j.at("error_message").as_string();
  s.has_camera_frame = j.at("has_camera_frame").as_bool();
  s.camera_frame_id = j.at("camera_frame_id").as_string();
  s.has_point_cloud = j.at("has_point_cloud").as_bool();
  s.point_cloud_id = j.at("point_cloud_id").as_string();
  if (const json::Value* events = j.find("events"); events && events->is_array()) {
    for (const auto& e : events->as_array()) {
      sdk::ControllerEvent ev;
      ev.timestamp = core::Timestamp::from_nanoseconds(e.at("t_ns").as_int());
      ev.kind = machine::event_kind_from_string(e.at("kind").as_string("warning"));
      ev.severity = machine::severity_from_string(e.at("severity").as_string("info"));
      ev.message = e.at("message").as_string();
      s.events.push_back(std::move(ev));
    }
  }
  return s;
}

// -------------------------------------------------------------------- commands

[[nodiscard]] inline std::string command_line(std::string_view cmd) {
  json::Value j;
  j.set("cmd", std::string(cmd));
  return j.dump(0);
}

[[nodiscard]] inline std::string load_task_line(const machine::MotionTask& task) {
  json::Value j;
  j.set("cmd", "load_task");
  j.set("task", task_to_json(task));
  return j.dump(0);
}

// Immediate move (jog / teleoperation from the scene): one motion command to run
// now, outside the loaded program.
[[nodiscard]] inline std::string move_to_line(const machine::MotionCommand& command) {
  json::Value j;
  j.set("cmd", "move_to");
  j.set("command", command_to_json(command));
  return j.dump(0);
}

// The reply/telemetry `type` field.
[[nodiscard]] inline std::string message_type(const json::Value& j) {
  return j.at("type").as_string();
}

}  // namespace cavr::adapters::generic_tcp_robot::protocol
