#pragma once

// Reference recording backend: serializes a Recording to the project's
// hand-rolled JSON (cavr::json, the same value type used for session and profile
// I/O) and reads it back. It is intentionally simple and non-streaming — it is
// the dependency-free baseline that proves the RecordingWriter/RecordingReader
// interfaces. The authoritative MCAP backend will implement the same interfaces.

#include <cavr/machine/json.hpp>
#include <cavr/record/reader.hpp>
#include <cavr/record/writer.hpp>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>

namespace cavr::record {

[[nodiscard]] inline json::Value to_json(const Recording& recording) {
  json::Value root;
  root.set("version", kRecordingFormatVersion);

  json::Array channels;
  for (const auto& c : recording.channels) {
    json::Value j;
    j.set("id", static_cast<std::int64_t>(c.id));
    j.set("topic", c.topic);
    j.set("content_type", c.content_type);
    j.set("schema_name", c.schema_name);
    channels.push_back(std::move(j));
  }
  root.set("channels", std::move(channels));

  json::Array messages;
  for (const auto& m : recording.messages) {
    json::Value j;
    j.set("channel", static_cast<std::int64_t>(m.channel_id));
    j.set("t_ns", static_cast<std::int64_t>(m.log_time.nanoseconds()));
    j.set("data", m.data);
    messages.push_back(std::move(j));
  }
  root.set("messages", std::move(messages));
  return root;
}

[[nodiscard]] inline RecordStatus save_recording(const Recording& recording,
                                                 const std::filesystem::path& path) {
  std::ofstream out(path);
  if (!out) {
    return RecordStatus::failure("Failed to open recording for writing: " + path.string());
  }
  out << to_json(recording).dump(1) << '\n';
  return RecordStatus::success();
}

// Result of loading a recording. On failure `recording` is empty and `status`
// carries the reason.
struct RecordingLoad final {
  Recording recording;
  RecordStatus status;
};

[[nodiscard]] inline RecordingLoad load_recording(const std::filesystem::path& path) {
  RecordingLoad result;

  std::ifstream in(path);
  if (!in) {
    result.status = RecordStatus::failure("Failed to open recording: " + path.string());
    return result;
  }

  std::ostringstream buffer;
  buffer << in.rdbuf();
  std::string parse_error;
  const auto root = json::parse(buffer.str(), parse_error);
  if (!root) {
    result.status = RecordStatus::failure("Invalid recording JSON: " + parse_error);
    return result;
  }

  const std::int64_t version = root->at("version").as_int(kRecordingFormatVersion);
  if (version > kRecordingFormatVersion) {
    result.status = RecordStatus::failure(
        "Unsupported recording format version " + std::to_string(version) + " (this build supports " +
        std::to_string(kRecordingFormatVersion) + ")");
    return result;
  }

  if (const json::Value* channels = root->find("channels"); channels && channels->is_array()) {
    for (const auto& j : channels->as_array()) {
      Channel c;
      c.id = static_cast<ChannelId>(j.at("id").as_int());
      c.topic = j.at("topic").as_string();
      c.content_type = j.at("content_type").as_string();
      c.schema_name = j.at("schema_name").as_string();
      result.recording.channels.push_back(std::move(c));
    }
  }

  if (const json::Value* messages = root->find("messages"); messages && messages->is_array()) {
    for (const auto& j : messages->as_array()) {
      Message m;
      m.channel_id = static_cast<ChannelId>(j.at("channel").as_int());
      m.log_time = core::Timestamp::from_nanoseconds(j.at("t_ns").as_int());
      m.data = j.at("data").as_string();
      result.recording.messages.push_back(std::move(m));
    }
  }

  result.status = RecordStatus::success();
  return result;
}

// Writer backed by a JSON file. Messages are buffered in memory and the file is
// written on close(); this is adequate for the reference backend (it matches how
// session logs are persisted) and keeps the implementation dependency-free.
class JsonRecordingWriter final : public RecordingWriter {
 public:
  explicit JsonRecordingWriter(std::filesystem::path path) : path_(std::move(path)) {}

  ChannelId add_channel(const Channel& channel) override {
    return recording_.add_channel(channel);
  }

  RecordStatus write(const Message& message) override {
    if (closed_) return RecordStatus::failure("Recording writer is already closed");
    if (recording_.channel_by_id(message.channel_id) == nullptr) {
      return RecordStatus::failure("Message references unknown channel id " +
                                   std::to_string(message.channel_id));
    }
    recording_.messages.push_back(message);
    return RecordStatus::success();
  }

  RecordStatus close() override {
    if (closed_) return RecordStatus::success();
    const RecordStatus status = save_recording(recording_, path_);
    if (status) closed_ = true;
    return status;
  }

  [[nodiscard]] const Recording& recording() const noexcept { return recording_; }

 private:
  std::filesystem::path path_;
  Recording recording_;
  bool closed_{false};
};

// Reader serving a Recording loaded from JSON. Construct it from the data
// returned by load_recording().
class JsonRecordingReader final : public RecordingReader {
 public:
  explicit JsonRecordingReader(Recording recording) : recording_(std::move(recording)) {}

  const std::vector<Channel>& channels() const override { return recording_.channels; }

  const Channel* find_channel(std::string_view topic) const override {
    return recording_.find_channel(topic);
  }

  std::vector<Message> messages() const override { return recording_.sorted_by_time(); }

  std::vector<Message> messages_on(ChannelId channel) const override {
    return recording_.on_channel(channel);
  }

 private:
  Recording recording_;
};

}  // namespace cavr::record
