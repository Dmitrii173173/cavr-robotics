// Synchronized robot + point-cloud recording: drive a real session with a mock
// controller and a mock camera (which now also emits a synthetic scan cloud each
// tick) attached to the SessionManager. Each tick captures telemetry and a point
// cloud on the same clock, streamed to the backend. Afterwards the recording must
// carry the cloud stream, time-aligned with telemetry, and read back verbatim.

#include <cavr/adapters/mock_camera/mock_camera.hpp>
#include <cavr/adapters/mock_robot/mock_controller.hpp>
#include <cavr/record/json_recording.hpp>
#include <cavr/runtime/demo_plan.hpp>
#include <cavr/runtime/point_cloud_recording.hpp>
#include <cavr/runtime/record_session.hpp>
#include <cavr/runtime/session_manager.hpp>
#include <cavr/runtime/session_recorder.hpp>

#ifdef CAVR_WITH_MCAP
#include <cavr/storage_mcap/mcap_recording.hpp>
#endif

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <set>
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

namespace runtime = cavr::runtime;
namespace record = cavr::record;
namespace mock = cavr::adapters::mock_robot;
namespace mock_cam = cavr::adapters::mock_camera;
namespace sdk = cavr::adapter_sdk;

std::filesystem::path temp_path(std::string_view name) {
  return std::filesystem::temp_directory_path() / name;
}

runtime::SessionLog run_robot_and_scan(runtime::SessionRecorder& recorder) {
  mock::MockController controller;
  mock_cam::MockCamera camera(8, 8, "weld_cam");
  runtime::SessionManager manager;
  manager.attach_recorder(recorder);
  manager.attach_camera(camera);

  (void)manager.connect(controller, {"mock", "mock"});
  (void)manager.discover_profile();
  manager.set_plan(runtime::make_demo_plan());
  (void)manager.validate();
  check(manager.execute("robot_scan_session"), "session starts executing");

  std::int64_t now_ns = 1'000'000'000;
  int ticks = 0;
  while (manager.phase() == runtime::SessionPhase::Executing && ticks < 5000) {
    manager.tick(cavr::core::Timestamp::from_nanoseconds(now_ns));
    now_ns += 20'000'000;  // 50 Hz
    ++ticks;
  }
  check(manager.has_point_cloud(), "manager exposes the latest point cloud");
  check(manager.latest_point_cloud().size() == 16, "latest cloud is the 4x4 mock grid");

  check(static_cast<bool>(recorder.finish(manager.log())), "recorder finalizes");
  return manager.log();
}

void check_synchronized(const runtime::SessionLog& log, const runtime::SessionRecorder& recorder,
                        record::RecordingReader& reader, std::string_view backend) {
  const std::string ctx = std::string(backend) + ": ";

  check(recorder.errors() == 0, (ctx + "no write errors").c_str());
  check(recorder.point_clouds_written() == log.frame_count(),
        (ctx + "one point cloud per telemetry tick").c_str());

  check(reader.find_channel(record::topics::kCameraPoints) != nullptr,
        (ctx + "point-cloud channel present").c_str());

  const auto clouds = runtime::read_point_clouds(reader);
  check(clouds.size() == log.frame_count(), (ctx + "all point clouds round-trip").c_str());

  // Robot and scan streams share one clock: every cloud timestamp is a telemetry
  // instant.
  std::set<std::int64_t> telemetry_times;
  for (const auto& f : log.frames) telemetry_times.insert(f.timestamp.nanoseconds());
  bool aligned = !clouds.empty();
  for (const auto& c : clouds) {
    if (telemetry_times.find(c.timestamp.nanoseconds()) == telemetry_times.end()) aligned = false;
  }
  check(aligned, (ctx + "point clouds are time-aligned with telemetry").c_str());

  // Geometry round-trips: the first cloud is the deterministic 4x4 grid with a
  // color per point. Reconstruct its first point and compare.
  const sdk::PointCloud& first = clouds.front();
  check(first.frame_id == "weld_cam", (ctx + "cloud frame id round-trips").c_str());
  check(first.size() == 16, (ctx + "cloud point count round-trips").c_str());
  check(first.has_colors(), (ctx + "per-point colors round-trip").c_str());

  const double phase = static_cast<double>((first.timestamp.nanoseconds() / 1'000'000) & 0xFF) * 0.01;
  const double x0 = (0.0 / 4 - 0.5) * 0.2;
  const double y0 = (0.0 / 4 - 0.5) * 0.2;
  const double z0 = 0.5 + 0.01 * std::sin(phase + 0 + 0);
  check(std::abs(first.points.front().x_m - x0) < 1e-9 &&
            std::abs(first.points.front().y_m - y0) < 1e-9 &&
            std::abs(first.points.front().z_m - z0) < 1e-9,
        (ctx + "cloud point coordinates round-trip verbatim").c_str());
}

void test_json() {
  const auto path = temp_path("cavr_robot_scan.json");
  std::filesystem::remove(path);

  record::JsonRecordingWriter writer(path);
  runtime::SessionRecorder recorder(writer);
  const runtime::SessionLog log = run_robot_and_scan(recorder);
  check(log.frame_count() > 10, "json: telemetry recorded");

  const auto loaded = record::load_recording(path);
  check(static_cast<bool>(loaded.status), "json: recording loads");
  record::JsonRecordingReader reader(loaded.recording);
  check_synchronized(log, recorder, reader, "json");

  std::filesystem::remove(path);
}

#ifdef CAVR_WITH_MCAP
void test_mcap() {
  namespace storage_mcap = cavr::storage_mcap;
  const auto path = temp_path("cavr_robot_scan.mcap");
  std::filesystem::remove(path);

  storage_mcap::McapRecordingWriter writer(path, /*streaming=*/true);
  runtime::SessionRecorder recorder(writer);
  const runtime::SessionLog log = run_robot_and_scan(recorder);

  const auto loaded = storage_mcap::load_recording(path);
  check(static_cast<bool>(loaded.status), "mcap: recording loads");
  storage_mcap::McapRecordingReader reader(loaded.recording);
  check_synchronized(log, recorder, reader, "mcap");

  std::filesystem::remove(path);
}
#endif

}  // namespace

int main() {
  test_json();
#ifdef CAVR_WITH_MCAP
  test_mcap();
#endif

  if (failures != 0) {
    std::cerr << failures << " point cloud recording test(s) failed\n";
    return 1;
  }
  std::cout << "runtime point cloud recording tests passed\n";
  return 0;
}
