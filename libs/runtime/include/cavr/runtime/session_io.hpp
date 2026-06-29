#pragma once

// Persist a SessionLog as JSON for replay and diagnostics, and read it back. The
// profile is serialized in full; telemetry frames are serialized compactly
// (enough to drive the viewport, timeline and status panels on replay).

#include <cavr/machine/json.hpp>
#include <cavr/machine/profile_io.hpp>
#include <cavr/runtime/session.hpp>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace cavr::runtime {

namespace detail {

[[nodiscard]] inline json::Value frame_to_json(const sdk::RobotState& f) {
  json::Value j;
  j.set("t_ns", static_cast<std::int64_t>(f.timestamp.nanoseconds()));
  json::Array joints;
  for (double v : f.joint_positions) joints.push_back(v);
  j.set("joints", std::move(joints));
  j.set("tcp", machine::detail::pose_to_json(f.tcp_pose));
  j.set("program_state", machine::to_string(f.program_state));
  j.set("servo_state", machine::to_string(f.servo_state));
  j.set("step", f.current_step);
  j.set("step_label", f.current_step_label);
  j.set("speed_fraction", f.speed_fraction);
  j.set("tcp_speed_mm_s", f.tcp_speed_mm_s);
  json::Array io;
  for (const auto& s : f.io) {
    json::Value e;
    e.set("name", s.name);
    e.set("value", s.value);
    io.push_back(std::move(e));
  }
  j.set("io", std::move(io));
  j.set("error_severity", machine::to_string(f.error_severity));
  j.set("error_code", f.error_code);
  j.set("error_message", f.error_message);
  return j;
}

[[nodiscard]] inline sdk::RobotState frame_from_json(const json::Value& j) {
  sdk::RobotState f;
  f.timestamp = core::Timestamp::from_nanoseconds(j.at("t_ns").as_int());
  if (const json::Value* joints = j.find("joints"); joints && joints->is_array()) {
    for (const auto& v : joints->as_array()) f.joint_positions.push_back(v.as_number());
  }
  f.tcp_pose = machine::detail::pose_from_json(j.at("tcp"));
  f.program_state = machine::program_state_from_string(j.at("program_state").as_string("idle"));
  f.servo_state = machine::servo_state_from_string(j.at("servo_state").as_string("off"));
  f.current_step = static_cast<int>(j.at("step").as_int(-1));
  f.current_step_label = j.at("step_label").as_string();
  f.speed_fraction = j.at("speed_fraction").as_number();
  f.tcp_speed_mm_s = j.at("tcp_speed_mm_s").as_number();
  if (const json::Value* io = j.find("io"); io && io->is_array()) {
    for (const auto& e : io->as_array())
      f.io.push_back({e.at("name").as_string(), e.at("value").as_number()});
  }
  f.error_severity = machine::severity_from_string(j.at("error_severity").as_string("info"));
  f.error_code = static_cast<int>(j.at("error_code").as_int());
  f.error_message = j.at("error_message").as_string();
  return f;
}

[[nodiscard]] inline json::Value event_to_json(const TimelineEvent& e) {
  json::Value j;
  j.set("t_ns", static_cast<std::int64_t>(e.timestamp.nanoseconds()));
  j.set("kind", machine::to_string(e.kind));
  j.set("severity", machine::to_string(e.severity));
  j.set("message", e.message);
  j.set("step_index", e.step_index);
  return j;
}

}  // namespace detail

[[nodiscard]] inline json::Value to_json(const SessionLog& log) {
  json::Value root;
  root.set("session_id", log.session_id);
  root.set("started_ns", static_cast<std::int64_t>(log.started.nanoseconds()));
  root.set("ended_ns", static_cast<std::int64_t>(log.ended.nanoseconds()));
  root.set("profile", machine::to_json(log.profile));

  json::Array steps;
  for (const auto& s : log.timeline.steps) {
    json::Value j;
    j.set("id", s.id);
    j.set("kind", to_string(s.kind));
    j.set("label", s.label);
    j.set("planned_duration_s", s.planned_duration_s);
    j.set("notes", s.notes);
    steps.push_back(std::move(j));
  }
  root.set("steps", std::move(steps));

  json::Array events;
  for (const auto& e : log.timeline.events) events.push_back(detail::event_to_json(e));
  root.set("events", std::move(events));

  json::Array frames;
  for (const auto& f : log.frames) frames.push_back(detail::frame_to_json(f));
  root.set("frames", std::move(frames));
  return root;
}

[[nodiscard]] inline bool save_session_log(const SessionLog& log, const std::filesystem::path& path,
                                           std::vector<std::string>& errors) {
  std::ofstream out(path);
  if (!out) {
    errors.push_back("Failed to open session log for writing: " + path.string());
    return false;
  }
  out << to_json(log).dump(1) << '\n';
  return true;
}

struct SessionLoadResult final {
  SessionLog log;
  std::vector<std::string> errors;
  [[nodiscard]] bool ok() const noexcept { return errors.empty(); }
};

[[nodiscard]] inline SessionLoadResult load_session_log(const std::filesystem::path& path) {
  SessionLoadResult result;
  std::ifstream in(path);
  if (!in) {
    result.errors.push_back("Failed to open session log: " + path.string());
    return result;
  }
  std::ostringstream buffer;
  buffer << in.rdbuf();
  std::string parse_error;
  auto value = json::parse(buffer.str(), parse_error);
  if (!value) {
    result.errors.push_back("Invalid session JSON: " + parse_error);
    return result;
  }
  const json::Value& root = *value;
  result.log.session_id = root.at("session_id").as_string();
  result.log.started = core::Timestamp::from_nanoseconds(root.at("started_ns").as_int());
  result.log.ended = core::Timestamp::from_nanoseconds(root.at("ended_ns").as_int());
  result.log.profile = machine::profile_from_json(root.at("profile"));

  if (const json::Value* steps = root.find("steps"); steps && steps->is_array()) {
    for (const auto& j : steps->as_array()) {
      OperationStep s;
      s.id = static_cast<int>(j.at("id").as_int());
      s.kind = operation_kind_from_string(j.at("kind").as_string("robot_motion"));
      s.label = j.at("label").as_string();
      s.planned_duration_s = j.at("planned_duration_s").as_number();
      s.notes = j.at("notes").as_string();
      result.log.timeline.steps.push_back(std::move(s));
    }
  }
  if (const json::Value* events = root.find("events"); events && events->is_array()) {
    for (const auto& j : events->as_array()) {
      TimelineEvent e;
      e.timestamp = core::Timestamp::from_nanoseconds(j.at("t_ns").as_int());
      e.kind = machine::event_kind_from_string(j.at("kind").as_string("warning"));
      e.severity = machine::severity_from_string(j.at("severity").as_string("info"));
      e.message = j.at("message").as_string();
      e.step_index = static_cast<int>(j.at("step_index").as_int(-1));
      result.log.timeline.events.push_back(std::move(e));
    }
  }
  if (const json::Value* frames = root.find("frames"); frames && frames->is_array()) {
    for (const auto& j : frames->as_array()) result.log.frames.push_back(detail::frame_from_json(j));
  }
  return result;
}

}  // namespace cavr::runtime
