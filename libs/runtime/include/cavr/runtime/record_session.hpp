#pragma once

// Bridges a SessionLog onto the storage-neutral recording interfaces:
//
//   telemetry frames   -> messages on  robot/telemetry
//   controller events  -> messages on  controller/events
//   session header     -> one message on session/meta  (id, profile, plan, span)
//
// This is the seam that makes the record layer real: a whole session is persisted
// and replayed through any backend — the JSON reference backend or MCAP — and the
// runtime never knows which. write_session() is the Monitor-phase sink;
// read_session() is the Replay-phase source.
//
// Per-frame and per-event payloads reuse the same compact JSON the session log
// already uses (session_io.hpp), so the on-the-wire shape stays consistent.

#include <cavr/machine/json.hpp>
#include <cavr/machine/profile_io.hpp>
#include <cavr/record/reader.hpp>
#include <cavr/record/writer.hpp>
#include <cavr/runtime/session.hpp>
#include <cavr/runtime/session_io.hpp>

#include <string>
#include <string_view>

namespace cavr::runtime {

namespace detail {

inline constexpr std::string_view kSessionMetaTopic = "session/meta";
inline constexpr std::string_view kSessionMetaSchema = "cavr.runtime.SessionHeader";

// The non-timeseries part of a session: identity, profile and the planned steps.
// (Events and frames travel as their own message streams.)
[[nodiscard]] inline json::Value session_header_to_json(const SessionLog& log) {
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
  return root;
}

inline void apply_session_header(const json::Value& root, SessionLog& log) {
  log.session_id = root.at("session_id").as_string();
  log.started = core::Timestamp::from_nanoseconds(root.at("started_ns").as_int());
  log.ended = core::Timestamp::from_nanoseconds(root.at("ended_ns").as_int());
  log.profile = machine::profile_from_json(root.at("profile"));
  if (const json::Value* steps = root.find("steps"); steps && steps->is_array()) {
    for (const auto& j : steps->as_array()) {
      OperationStep s;
      s.id = static_cast<int>(j.at("id").as_int());
      s.kind = operation_kind_from_string(j.at("kind").as_string("robot_motion"));
      s.label = j.at("label").as_string();
      s.planned_duration_s = j.at("planned_duration_s").as_number();
      s.notes = j.at("notes").as_string();
      log.timeline.steps.push_back(std::move(s));
    }
  }
}

}  // namespace detail

// Writes a complete session through a recording backend and finalizes it. Stops
// and returns the first failure encountered.
[[nodiscard]] inline record::RecordStatus write_session(const SessionLog& log,
                                                        record::RecordingWriter& writer) {
  const record::ChannelId meta = writer.add_channel(
      {0, std::string(detail::kSessionMetaTopic), std::string(record::content_type::kJson),
       std::string(detail::kSessionMetaSchema)});
  const record::ChannelId telemetry = writer.add_channel(
      {0, std::string(record::topics::kRobotTelemetry), std::string(record::content_type::kJson),
       std::string(record::schemas::kRobotState)});
  const record::ChannelId events = writer.add_channel(
      {0, std::string(record::topics::kControllerEvents), std::string(record::content_type::kJson),
       std::string(record::schemas::kControllerEvent)});

  if (record::RecordStatus s = writer.write({meta, log.started, detail::session_header_to_json(log).dump(0)});
      !s) {
    return s;
  }
  for (const auto& f : log.frames) {
    if (record::RecordStatus s = writer.write({telemetry, f.timestamp, detail::frame_to_json(f).dump(0)});
        !s) {
      return s;
    }
  }
  for (const auto& e : log.timeline.events) {
    if (record::RecordStatus s = writer.write({events, e.timestamp, detail::event_to_json(e).dump(0)});
        !s) {
      return s;
    }
  }
  return writer.close();
}

struct SessionRecordingResult final {
  SessionLog log;
  record::RecordStatus status;
  [[nodiscard]] bool ok() const noexcept { return status.ok; }
};

// Reconstructs a SessionLog from a recording. Missing streams yield empty parts
// rather than errors; only malformed payloads fail.
[[nodiscard]] inline SessionRecordingResult read_session(const record::RecordingReader& reader) {
  SessionRecordingResult result;

  auto parse_payload = [&](const record::Message& m, json::Value& out) -> bool {
    std::string error;
    auto value = json::parse(m.data, error);
    if (!value) {
      result.status = record::RecordStatus::failure("Invalid session payload JSON: " + error);
      return false;
    }
    out = std::move(*value);
    return true;
  };

  if (const record::Channel* meta = reader.find_channel(detail::kSessionMetaTopic)) {
    const auto messages = reader.messages_on(meta->id);
    if (!messages.empty()) {
      json::Value header;
      if (!parse_payload(messages.front(), header)) return result;
      detail::apply_session_header(header, result.log);
    }
  }

  if (const record::Channel* telemetry = reader.find_channel(record::topics::kRobotTelemetry)) {
    for (const auto& m : reader.messages_on(telemetry->id)) {
      json::Value frame;
      if (!parse_payload(m, frame)) return result;
      result.log.frames.push_back(detail::frame_from_json(frame));
    }
  }

  if (const record::Channel* events = reader.find_channel(record::topics::kControllerEvents)) {
    for (const auto& m : reader.messages_on(events->id)) {
      json::Value j;
      if (!parse_payload(m, j)) return result;
      TimelineEvent e;
      e.timestamp = core::Timestamp::from_nanoseconds(j.at("t_ns").as_int());
      e.kind = machine::event_kind_from_string(j.at("kind").as_string("warning"));
      e.severity = machine::severity_from_string(j.at("severity").as_string("info"));
      e.message = j.at("message").as_string();
      e.step_index = static_cast<int>(j.at("step_index").as_int(-1));
      result.log.timeline.events.push_back(std::move(e));
    }
  }

  result.status = record::RecordStatus::success();
  return result;
}

}  // namespace cavr::runtime
