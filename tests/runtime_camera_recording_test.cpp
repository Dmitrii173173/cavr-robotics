// Synchronized robot + camera recording: drive a real session with both a mock
// controller and a mock camera attached to the SessionManager. Each tick captures
// a telemetry frame and a camera frame on the same clock, streamed to the backend.
// Afterwards the recording must carry both streams, time-aligned, and read back.

#include <cavr/adapters/mock_camera/mock_camera.hpp>
#include <cavr/adapters/mock_robot/mock_controller.hpp>
#include <cavr/record/json_recording.hpp>
#include <cavr/runtime/camera_recording.hpp>
#include <cavr/runtime/demo_plan.hpp>
#include <cavr/runtime/record_session.hpp>
#include <cavr/runtime/session_manager.hpp>
#include <cavr/runtime/session_recorder.hpp>

#ifdef CAVR_WITH_MCAP
#include <cavr/storage_mcap/mcap_recording.hpp>
#endif

#include <cstdint>
#include <filesystem>
#include <iostream>
#include <set>
#include <string>
#include <string_view>
#include <vector>

namespace {

int failures = 0;

void check(bool condition, std::string_view message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
    ++failures;
  }
}

namespace runtime = cavr::runtime;
namespace record = cavr::record;
namespace mock = cavr::adapters::mock_robot;
namespace mock_cam = cavr::adapters::mock_camera;
namespace sdk = cavr::adapter_sdk;

std::filesystem::path temp_path(std::string_view name) {
  return std::filesystem::temp_directory_path() / name;
}

// Drives the demo session with telemetry + camera attached and finalizes it.
runtime::SessionLog run_robot_and_camera(runtime::SessionRecorder& recorder) {
  mock::MockController controller;
  mock_cam::MockCamera camera(8, 8, "weld_cam");
  runtime::SessionManager manager;
  manager.attach_recorder(recorder);
  manager.attach_camera(camera);

  (void)manager.connect(controller, {"mock", "mock"});
  (void)manager.discover_profile();
  manager.set_plan(runtime::make_demo_plan());
  (void)manager.validate();
  check(manager.execute("robot_camera_session"), "session starts executing");

  std::int64_t now_ns = 1'000'000'000;
  int ticks = 0;
  while (manager.phase() == runtime::SessionPhase::Executing && ticks < 5000) {
    manager.tick(cavr::core::Timestamp::from_nanoseconds(now_ns));
    now_ns += 20'000'000;  // 50 Hz
    ++ticks;
  }
  check(manager.has_camera_frame(), "manager exposes the latest camera frame");

  check(static_cast<bool>(recorder.finish(manager.log())), "recorder finalizes");
  return manager.log();
}

void check_synchronized(const runtime::SessionLog& log, const runtime::SessionRecorder& recorder,
                        record::RecordingReader& reader, std::string_view backend) {
  const std::string ctx = std::string(backend) + ": ";

  check(recorder.errors() == 0, (ctx + "no write errors").c_str());
  check(recorder.frames_written() == log.frame_count(), (ctx + "telemetry fully streamed").c_str());
  check(recorder.camera_frames_written() == log.frame_count(),
        (ctx + "one camera frame per telemetry tick").c_str());

  check(reader.find_channel(record::topics::kRobotTelemetry) != nullptr,
        (ctx + "telemetry channel present").c_str());
  check(reader.find_channel(record::topics::kCameraColor) != nullptr,
        (ctx + "camera channel present").c_str());

  const auto camera_frames = runtime::read_camera_frames(reader);
  check(camera_frames.size() == log.frame_count(), (ctx + "all camera frames round-trip").c_str());

  // Robot and camera streams share one clock: the telemetry timestamps and camera
  // timestamps must be the same set of instants.
  std::set<std::int64_t> telemetry_times;
  for (const auto& f : log.frames) telemetry_times.insert(f.timestamp.nanoseconds());
  bool aligned = !camera_frames.empty();
  for (const auto& cf : camera_frames) {
    if (telemetry_times.find(cf.timestamp.nanoseconds()) == telemetry_times.end()) aligned = false;
  }
  check(aligned, (ctx + "camera frames are time-aligned with telemetry").c_str());

  // Pixel payload round-trips: the first frame is the deterministic mock image.
  const sdk::CameraFrame& first = camera_frames.front();
  check(first.width == 8 && first.height == 8 && first.encoding == "mono8",
        (ctx + "camera frame geometry round-trips").c_str());
  check(first.pixels.size() == 64, (ctx + "camera pixel buffer round-trips").c_str());
  const auto seed = static_cast<std::uint8_t>((first.timestamp.nanoseconds() / 1'000'000) & 0xFF);
  check(first.pixels.front() == seed, (ctx + "camera pixel values round-trip verbatim").c_str());
}

void test_robot_camera_json() {
  const auto path = temp_path("cavr_robot_camera.json");
  std::filesystem::remove(path);

  record::JsonRecordingWriter writer(path);
  runtime::SessionRecorder recorder(writer);
  const runtime::SessionLog log = run_robot_and_camera(recorder);
  check(log.frame_count() > 10, "json: telemetry recorded");

  const auto loaded = record::load_recording(path);
  check(static_cast<bool>(loaded.status), "json: recording loads");
  record::JsonRecordingReader reader(loaded.recording);
  check_synchronized(log, recorder, reader, "json");

  std::filesystem::remove(path);
}

#ifdef CAVR_WITH_MCAP
void test_robot_camera_mcap() {
  namespace storage_mcap = cavr::storage_mcap;
  const auto path = temp_path("cavr_robot_camera.mcap");
  std::filesystem::remove(path);

  storage_mcap::McapRecordingWriter writer(path, /*streaming=*/true);
  runtime::SessionRecorder recorder(writer);
  const runtime::SessionLog log = run_robot_and_camera(recorder);

  const auto loaded = storage_mcap::load_recording(path);
  check(static_cast<bool>(loaded.status), "mcap: recording loads");
  storage_mcap::McapRecordingReader reader(loaded.recording);
  check_synchronized(log, recorder, reader, "mcap");

  std::filesystem::remove(path);
}
#endif

}  // namespace

int main() {
  test_robot_camera_json();
#ifdef CAVR_WITH_MCAP
  test_robot_camera_mcap();
#endif

  if (failures != 0) {
    std::cerr << failures << " camera recording test(s) failed\n";
    return 1;
  }
  std::cout << "runtime camera recording tests passed\n";
  return 0;
}
