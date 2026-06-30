#include <cavr/record/json_recording.hpp>
#include <cavr/record/reader.hpp>
#include <cavr/record/writer.hpp>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <string_view>

namespace {

int failures = 0;

void check(bool condition, std::string_view message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
    ++failures;
  }
}

namespace record = cavr::record;
using cavr::core::Timestamp;

std::filesystem::path temp_recording_path(std::string_view name) {
  return std::filesystem::temp_directory_path() / name;
}

// The in-memory Recording is the storage-neutral core: channel id assignment,
// topic lookup and time ordering must hold regardless of any backend.
void test_in_memory_recording() {
  record::Recording rec;
  const record::ChannelId telemetry =
      rec.add_channel({0, std::string(record::topics::kRobotTelemetry),
                       std::string(record::content_type::kJson), std::string(record::schemas::kRobotState)});
  const record::ChannelId events =
      rec.add_channel({0, std::string(record::topics::kControllerEvents),
                       std::string(record::content_type::kJson),
                       std::string(record::schemas::kControllerEvent)});

  check(telemetry == 1 && events == 2, "channel ids are assigned sequentially");
  check(rec.find_channel(record::topics::kRobotTelemetry) != nullptr, "channel found by topic");
  check(rec.find_channel("camera/depth") == nullptr, "missing topic returns nullptr");

  // Append out of timestamp order; the recording must hand them back sorted.
  rec.messages.push_back({telemetry, Timestamp::from_nanoseconds(200), "b"});
  rec.messages.push_back({events, Timestamp::from_nanoseconds(150), "evt"});
  rec.messages.push_back({telemetry, Timestamp::from_nanoseconds(100), "a"});

  const auto ordered = rec.sorted_by_time();
  check(ordered.size() == 3, "all messages returned");
  check(ordered.front().log_time.nanoseconds() == 100 && ordered.back().log_time.nanoseconds() == 200,
        "messages are ordered by log_time");

  const auto telemetry_only = rec.on_channel(telemetry);
  check(telemetry_only.size() == 2, "channel filter keeps only its messages");
  check(telemetry_only.front().data == "a" && telemetry_only.back().data == "b",
        "filtered messages stay time-ordered");
}

// Full reference-backend round-trip exercised through the abstract interfaces,
// including a reserved camera channel to show the synchronized multi-stream shape.
void test_writer_reader_roundtrip() {
  const auto path = temp_recording_path("cavr_recording_roundtrip.json");
  std::filesystem::remove(path);

  record::ChannelId telemetry = 0;
  record::ChannelId camera = 0;
  {
    record::JsonRecordingWriter json_writer(path);
    record::RecordingWriter& writer = json_writer;  // drive via the neutral interface

    telemetry = writer.add_channel({0, std::string(record::topics::kRobotTelemetry),
                                    std::string(record::content_type::kJson),
                                    std::string(record::schemas::kRobotState)});
    camera = writer.add_channel({0, std::string(record::topics::kCameraColor),
                                 std::string(record::content_type::kJson), "cavr.CameraFrameRef"});

    check(static_cast<bool>(writer.write({telemetry, Timestamp::from_nanoseconds(100), R"({"joint":0.1})"})),
          "write telemetry message");
    check(static_cast<bool>(writer.write({camera, Timestamp::from_nanoseconds(120), R"({"frame":"f0001"})"})),
          "write camera reference message");
    check(static_cast<bool>(writer.write({telemetry, Timestamp::from_nanoseconds(200), R"({"joint":0.2})"})),
          "write second telemetry message");
    check(static_cast<bool>(writer.close()), "writer closes and flushes");
  }

  check(std::filesystem::exists(path), "recording file was written");

  const auto loaded = record::load_recording(path);
  check(static_cast<bool>(loaded.status), "recording loads without error");

  record::JsonRecordingReader json_reader(loaded.recording);
  record::RecordingReader& reader = json_reader;  // read via the neutral interface

  check(reader.channels().size() == 2, "both channels round-trip");
  const record::Channel* tele_channel = reader.find_channel(record::topics::kRobotTelemetry);
  check(tele_channel != nullptr && tele_channel->schema_name == std::string(record::schemas::kRobotState),
        "telemetry channel metadata round-trips");

  const auto all = reader.messages();
  check(all.size() == 3, "all messages round-trip");
  check(all.front().log_time.nanoseconds() == 100 && all.back().log_time.nanoseconds() == 200,
        "messages return in time order across channels");
  check(all.front().data == R"({"joint":0.1})", "opaque payload round-trips verbatim");

  const auto tele_messages = reader.messages_on(telemetry);
  check(tele_messages.size() == 2, "telemetry channel has two messages");
  const auto camera_messages = reader.messages_on(camera);
  check(camera_messages.size() == 1 && camera_messages.front().data == R"({"frame":"f0001"})",
        "camera channel carries its reference payload");

  std::filesystem::remove(path);
}

// Writing to an unregistered channel is a controlled failure, not a crash.
void test_unknown_channel_write_fails() {
  const auto path = temp_recording_path("cavr_recording_unknown_channel.json");
  std::filesystem::remove(path);

  record::JsonRecordingWriter writer(path);
  const record::RecordStatus status = writer.write({42, Timestamp::from_nanoseconds(1), "x"});
  check(!status.ok, "writing an unknown channel id fails");
  check(!status.error.empty(), "failure carries a message");

  std::filesystem::remove(path);
}

// A missing file is reported as a controlled error with an empty recording.
void test_missing_file_returns_error() {
  const auto path = temp_recording_path("cavr_recording_does_not_exist.json");
  std::filesystem::remove(path);

  const auto loaded = record::load_recording(path);
  check(!loaded.status.ok, "loading a missing file fails");
  check(loaded.recording.messages.empty() && loaded.recording.channels.empty(),
        "failed load yields an empty recording");
}

// A recording from a newer format version is rejected with a clear error rather
// than being silently mis-read.
void test_unsupported_version_rejected() {
  const auto path = temp_recording_path("cavr_recording_future_version.json");
  {
    std::ofstream out(path);
    out << R"({"version": 999, "channels": [], "messages": []})" << '\n';
  }

  const auto loaded = record::load_recording(path);
  check(!loaded.status.ok, "future format version is rejected");
  check(loaded.status.error.find("version") != std::string::npos,
        "version error names the problem");

  std::filesystem::remove(path);
}

// An empty recording round-trips cleanly.
void test_empty_recording_roundtrip() {
  const auto path = temp_recording_path("cavr_recording_empty.json");
  std::filesystem::remove(path);

  {
    record::JsonRecordingWriter writer(path);
    check(static_cast<bool>(writer.close()), "empty recording closes");
  }

  const auto loaded = record::load_recording(path);
  check(static_cast<bool>(loaded.status), "empty recording loads");
  check(loaded.recording.channels.empty() && loaded.recording.messages.empty(),
        "empty recording has no channels or messages");

  std::filesystem::remove(path);
}

}  // namespace

int main() {
  test_in_memory_recording();
  test_writer_reader_roundtrip();
  test_unknown_channel_write_fails();
  test_missing_file_returns_error();
  test_unsupported_version_rejected();
  test_empty_recording_roundtrip();

  if (failures == 0) std::cout << "record recording tests passed\n";
  return failures == 0 ? 0 : 1;
}
