// cavr-inspect: dumps the contents of a recording (channels, message counts,
// session header, camera stream) through the storage-neutral RecordingReader,
// so a .mcap or .json recording can be diagnosed without a separate viewer.

#include <cavr/record/json_recording.hpp>
#include <cavr/runtime/camera_recording.hpp>
#include <cavr/runtime/record_session.hpp>

#ifdef CAVR_WITH_MCAP
#include <cavr/storage_mcap/mcap_recording.hpp>
#endif

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>

namespace {

namespace record = cavr::record;
namespace runtime = cavr::runtime;

void print_usage() {
  std::cout << "Usage: cavr-inspect <recording.mcap|recording.json>\n";
}

[[nodiscard]] double to_seconds(std::int64_t ns) { return static_cast<double>(ns) / 1'000'000'000.0; }

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    print_usage();
    return argc == 1 ? 0 : 2;
  }

  const std::filesystem::path path = argv[1];
  const std::string ext = path.extension().string();

  record::Recording recording;
  record::RecordStatus status;
  if (ext == ".mcap") {
#ifdef CAVR_WITH_MCAP
    auto loaded = cavr::storage_mcap::load_recording(path);
    status = loaded.status;
    recording = std::move(loaded.recording);
#else
    std::cerr << "error: this build was configured with CAVR_ENABLE_MCAP=OFF; cannot read .mcap files\n";
    return 2;
#endif
  } else if (ext == ".json") {
    auto loaded = record::load_recording(path);
    status = loaded.status;
    recording = std::move(loaded.recording);
  } else {
    std::cerr << "error: unsupported extension '" << ext << "' (expected .mcap or .json)\n";
    return 2;
  }

  if (!status) {
    std::cerr << "error: failed to read " << path.string() << ": " << status.error << '\n';
    return 1;
  }

  const record::JsonRecordingReader reader(recording);

  std::cout << path.string() << " (" << recording.messages.size() << " messages, "
            << recording.channels.size() << " channels)\n";
  for (const auto& channel : recording.channels) {
    const std::size_t count = reader.messages_on(channel.id).size();
    std::cout << "  " << std::left << std::setw(24) << channel.topic << std::setw(36) << channel.schema_name
              << count << " messages\n";
  }

  const runtime::SessionRecordingResult session = runtime::read_session(reader);
  if (session.ok() && !session.log.session_id.empty()) {
    std::cout << "\nSession: " << session.log.session_id << '\n'
              << "  robot:    " << session.log.profile.robot_model << '\n'
              << "  span:     " << std::fixed << std::setprecision(3)
              << to_seconds(session.log.started.nanoseconds()) << "s .. "
              << to_seconds(session.log.ended.nanoseconds()) << "s\n"
              << "  frames:   " << session.log.frames.size() << '\n'
              << "  events:   " << session.log.timeline.events.size() << '\n';
  }

  const auto cameras = runtime::read_camera_frames(reader);
  if (!cameras.empty()) {
    std::cout << "  camera:   " << cameras.front().frame_id << " (" << cameras.front().encoding << ", "
              << cameras.front().width << "x" << cameras.front().height << "), " << cameras.size()
              << " frames\n";
  }

  return 0;
}
