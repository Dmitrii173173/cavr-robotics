// record::write_recording: copying a Recording through a writer must preserve the
// channels and message stream while remapping channel ids. Exercised through the
// JSON reference backend always, and additionally round-tripped through MCAP when
// that backend is built — proving format conversion (the basis of cavr-convert)
// is lossless for the logical channel/message model.

#include <cavr/record/copy.hpp>
#include <cavr/record/json_recording.hpp>

#ifdef CAVR_WITH_MCAP
#include <cavr/storage_mcap/mcap_recording.hpp>
#endif

#include <filesystem>
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

std::filesystem::path temp_path(std::string_view name) {
  return std::filesystem::temp_directory_path() / name;
}

// A small recording with two channels whose ids are deliberately non-contiguous,
// so a naive copy that trusted source ids would break.
record::Recording make_source() {
  record::Recording rec;
  const auto telemetry = rec.add_channel({7, "robot/telemetry", "application/json", "cavr.RobotState"});
  const auto events = rec.add_channel({42, "controller/events", "application/json", "cavr.Event"});
  rec.messages.push_back({telemetry, cavr::core::Timestamp::from_nanoseconds(100), "frame-a"});
  rec.messages.push_back({events, cavr::core::Timestamp::from_nanoseconds(150), "event-x"});
  rec.messages.push_back({telemetry, cavr::core::Timestamp::from_nanoseconds(200), "frame-b"});
  return rec;
}

// Verifies a loaded recording carries the same logical content as the source,
// regardless of how channel ids were assigned by the backend.
void expect_matches_source(const record::RecordingReader& reader, std::string_view ctx) {
  const std::string p = std::string(ctx) + ": ";

  const record::Channel* telemetry = reader.find_channel("robot/telemetry");
  const record::Channel* events = reader.find_channel("controller/events");
  check(telemetry != nullptr, (p + "telemetry channel present").c_str());
  check(events != nullptr, (p + "events channel present").c_str());
  if (!telemetry || !events) return;

  check(telemetry->schema_name == "cavr.RobotState", (p + "telemetry schema preserved").c_str());

  const auto tel_msgs = reader.messages_on(telemetry->id);
  check(tel_msgs.size() == 2, (p + "two telemetry messages").c_str());
  check(tel_msgs.size() == 2 && tel_msgs[0].data == "frame-a" && tel_msgs[1].data == "frame-b",
        (p + "telemetry payloads preserved in time order").c_str());

  const auto ev_msgs = reader.messages_on(events->id);
  check(ev_msgs.size() == 1 && ev_msgs[0].data == "event-x", (p + "event payload preserved").c_str());

  // Messages on different channels must not collide after remapping.
  check(telemetry->id != events->id, (p + "channels have distinct ids").c_str());
}

void test_copy_json_to_json() {
  const record::Recording source = make_source();
  const auto path = temp_path("cavr_copy.json");
  std::filesystem::remove(path);

  record::JsonRecordingWriter writer(path);
  check(static_cast<bool>(record::write_recording(source, writer)), "json copy succeeds");

  const auto loaded = record::load_recording(path);
  check(static_cast<bool>(loaded.status), "json copy reloads");
  record::JsonRecordingReader reader(loaded.recording);
  expect_matches_source(reader, "json->json");

  std::filesystem::remove(path);
}

#ifdef CAVR_WITH_MCAP
void test_convert_json_mcap_round_trip() {
  namespace storage_mcap = cavr::storage_mcap;
  const record::Recording source = make_source();

  // JSON -> MCAP: MCAP assigns its own channel ids, so this exercises remapping.
  const auto mcap_path = temp_path("cavr_copy.mcap");
  std::filesystem::remove(mcap_path);
  storage_mcap::McapRecordingWriter mcap_writer(mcap_path);
  check(static_cast<bool>(record::write_recording(source, mcap_writer)), "json->mcap copy succeeds");

  const auto mcap_loaded = storage_mcap::load_recording(mcap_path);
  check(static_cast<bool>(mcap_loaded.status), "mcap reloads");
  storage_mcap::McapRecordingReader mcap_reader(mcap_loaded.recording);
  expect_matches_source(mcap_reader, "json->mcap");

  // MCAP -> JSON: the round trip must land back on the same logical content.
  const auto json_path = temp_path("cavr_copy_back.json");
  std::filesystem::remove(json_path);
  record::JsonRecordingWriter json_writer(json_path);
  check(static_cast<bool>(record::write_recording(mcap_loaded.recording, json_writer)),
        "mcap->json copy succeeds");

  const auto json_loaded = record::load_recording(json_path);
  check(static_cast<bool>(json_loaded.status), "json reloads");
  record::JsonRecordingReader json_reader(json_loaded.recording);
  expect_matches_source(json_reader, "mcap->json");

  std::filesystem::remove(mcap_path);
  std::filesystem::remove(json_path);
}
#endif

}  // namespace

int main() {
  test_copy_json_to_json();
#ifdef CAVR_WITH_MCAP
  test_convert_json_mcap_round_trip();
#endif

  if (failures != 0) {
    std::cerr << failures << " record copy test(s) failed\n";
    return 1;
  }
  std::cout << "record copy tests passed\n";
  return 0;
}
