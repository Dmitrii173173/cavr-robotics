#include <cavr/record/reader.hpp>
#include <cavr/record/writer.hpp>
#include <cavr/storage_mcap/mcap_recording.hpp>

#include <array>
#include <cstdint>
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
namespace mcap = cavr::storage_mcap;
using cavr::core::Timestamp;

std::filesystem::path temp_recording_path(std::string_view name) {
  return std::filesystem::temp_directory_path() / name;
}

// Writing produces a real .mcap file (correct magic) and the stream round-trips
// through the neutral interfaces, just like the JSON backend.
void test_mcap_roundtrip() {
  const auto path = temp_recording_path("cavr_mcap_roundtrip.mcap");
  std::filesystem::remove(path);

  record::ChannelId telemetry = 0;
  record::ChannelId camera = 0;
  {
    mcap::McapRecordingWriter mcap_writer(path);
    record::RecordingWriter& writer = mcap_writer;  // drive via the neutral interface

    telemetry = writer.add_channel({0, std::string(record::topics::kRobotTelemetry),
                                    std::string(record::content_type::kJson),
                                    std::string(record::schemas::kRobotState)});
    camera = writer.add_channel({0, std::string(record::topics::kCameraColor),
                                 std::string(record::content_type::kJson), "cavr.CameraFrameRef"});
    check(telemetry != record::kInvalidChannel && camera != record::kInvalidChannel,
          "channels registered with valid ids");

    check(static_cast<bool>(writer.write({telemetry, Timestamp::from_nanoseconds(100), R"({"joint":0.1})"})),
          "write telemetry message");
    check(static_cast<bool>(writer.write({camera, Timestamp::from_nanoseconds(120), R"({"frame":"f0001"})"})),
          "write camera reference message");
    check(static_cast<bool>(writer.write({telemetry, Timestamp::from_nanoseconds(200), R"({"joint":0.2})"})),
          "write second telemetry message");
    check(static_cast<bool>(writer.close()), "writer closes and finalizes the file");
  }

  check(std::filesystem::exists(path), "mcap file was written");

  // The file must be a real MCAP: magic is 0x89 'M' 'C' 'A' 'P' '0' 0x0D 0x0A.
  {
    std::ifstream in(path, std::ios::binary);
    std::array<unsigned char, 8> magic{};
    in.read(reinterpret_cast<char*>(magic.data()), magic.size());
    const std::array<unsigned char, 8> expected{0x89, 'M', 'C', 'A', 'P', '0', 0x0D, 0x0A};
    check(magic == expected, "file begins with the MCAP magic");
  }

  const auto loaded = mcap::load_recording(path);
  check(static_cast<bool>(loaded.status), "mcap file loads without error");

  mcap::McapRecordingReader mcap_reader(loaded.recording);
  record::RecordingReader& reader = mcap_reader;  // read via the neutral interface

  check(reader.channels().size() == 2, "both channels round-trip");
  const record::Channel* tele_channel = reader.find_channel(record::topics::kRobotTelemetry);
  check(tele_channel != nullptr && tele_channel->schema_name == std::string(record::schemas::kRobotState),
        "telemetry channel metadata round-trips");
  check(tele_channel != nullptr && tele_channel->content_type == std::string(record::content_type::kJson),
        "channel message encoding round-trips");

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

// Writing to an unregistered channel is a controlled failure.
void test_unknown_channel_write_fails() {
  const auto path = temp_recording_path("cavr_mcap_unknown_channel.mcap");
  std::filesystem::remove(path);

  mcap::McapRecordingWriter writer(path);
  const record::RecordStatus status = writer.write({42, Timestamp::from_nanoseconds(1), "x"});
  check(!status.ok, "writing an unknown channel id fails");
  writer.close();

  std::filesystem::remove(path);
}

// Opening a file that is not valid MCAP is a controlled error.
void test_invalid_file_returns_error() {
  const auto path = temp_recording_path("cavr_mcap_not_mcap.mcap");
  {
    std::ofstream out(path, std::ios::binary);
    out << "this is not an mcap file";
  }

  const auto loaded = mcap::load_recording(path);
  check(!loaded.status.ok, "loading a non-MCAP file fails");
  check(loaded.recording.messages.empty(), "failed load yields no messages");

  std::filesystem::remove(path);
}

}  // namespace

int main() {
  test_mcap_roundtrip();
  test_unknown_channel_write_fails();
  test_invalid_file_returns_error();

  if (failures == 0) std::cout << "storage_mcap recording tests passed\n";
  return failures == 0 ? 0 : 1;
}
