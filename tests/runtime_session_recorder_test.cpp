// Live recorder test: drive a real session against the mock controller with a
// SessionRecorder attached to the SessionManager, so telemetry is streamed to the
// backend frame by frame during execution. Afterwards the recording is read back
// and must match the in-memory SessionLog — proving the live path and the
// in-memory log stay in sync, through the JSON backend and (when built) MCAP.

#include <cavr/adapters/mock_robot/mock_controller.hpp>
#include <cavr/record/json_recording.hpp>
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

std::filesystem::path temp_path(std::string_view name) {
  return std::filesystem::temp_directory_path() / name;
}

// Runs the demo session with the recorder attached, finalizing it at the end.
// Returns the manager's in-memory log for comparison against the recording.
runtime::SessionLog run_recorded_session(runtime::SessionRecorder& recorder) {
  mock::MockController controller;
  runtime::SessionManager manager;
  manager.attach_recorder(recorder);

  (void)manager.connect(controller, {"mock", "mock"});
  (void)manager.discover_profile();
  manager.set_plan(runtime::make_demo_plan());
  (void)manager.validate();
  check(manager.execute("recorded_session"), "recorded session starts executing");

  std::int64_t now_ns = 1'000'000'000;
  int ticks = 0;
  while (manager.phase() == runtime::SessionPhase::Executing && ticks < 5000) {
    manager.tick(cavr::core::Timestamp::from_nanoseconds(now_ns));
    now_ns += 20'000'000;  // 50 Hz
    ++ticks;
  }

  check(static_cast<bool>(recorder.finish(manager.log())), "recorder finalizes the recording");
  return manager.log();
}

void check_round_trip(const runtime::SessionLog& original, const runtime::SessionRecorder& recorder,
                      const runtime::SessionRecordingResult& loaded, std::string_view backend) {
  const std::string ctx = std::string(backend) + ": ";
  check(recorder.errors() == 0, (ctx + "recorder reported no write errors").c_str());
  check(recorder.frames_written() == original.frame_count(),
        (ctx + "every telemetry frame was streamed").c_str());

  check(loaded.ok(), (ctx + "recording reads back without error").c_str());
  check(loaded.log.session_id == original.session_id, (ctx + "session id round-trips").c_str());
  check(loaded.log.frame_count() == original.frame_count(), (ctx + "frame count round-trips").c_str());
  check(loaded.log.profile.dof() == original.profile.dof(), (ctx + "profile round-trips").c_str());
  check(loaded.log.timeline.events.size() == original.timeline.events.size(),
        (ctx + "controller events round-trip").c_str());
  check(loaded.log.timeline.steps.size() == original.timeline.steps.size(),
        (ctx + "plan steps round-trip").c_str());

  runtime::ReplayCursor cursor(loaded.log);
  check(cursor.duration_s() > 0.0, (ctx + "replay has positive duration").c_str());
}

void test_recorder_json() {
  const auto path = temp_path("cavr_recorder.json");
  std::filesystem::remove(path);

  record::JsonRecordingWriter writer(path);
  runtime::SessionRecorder recorder(writer);
  const runtime::SessionLog original = run_recorded_session(recorder);
  check(original.frame_count() > 10, "json: demo session recorded telemetry");

  const auto loaded = record::load_recording(path);
  check(static_cast<bool>(loaded.status), "json: recording file loads");
  record::JsonRecordingReader reader(loaded.recording);

  check_round_trip(original, recorder, runtime::read_session(reader), "json");

  std::filesystem::remove(path);
}

#ifdef CAVR_WITH_MCAP
void test_recorder_mcap_streaming() {
  namespace storage_mcap = cavr::storage_mcap;
  const auto path = temp_path("cavr_recorder.mcap");
  std::filesystem::remove(path);

  storage_mcap::McapRecordingWriter writer(path, /*streaming=*/true);
  runtime::SessionRecorder recorder(writer);
  const runtime::SessionLog original = run_recorded_session(recorder);

  check(std::filesystem::exists(path), "mcap: streaming recording file exists");
  check(std::filesystem::file_size(path) > 0, "mcap: streaming recording is non-empty");

  const auto loaded = storage_mcap::load_recording(path);
  check(static_cast<bool>(loaded.status), "mcap: streaming .mcap loads");
  storage_mcap::McapRecordingReader reader(loaded.recording);

  check_round_trip(original, recorder, runtime::read_session(reader), "mcap");

  std::filesystem::remove(path);
}
#endif

}  // namespace

int main() {
  test_recorder_json();
#ifdef CAVR_WITH_MCAP
  test_recorder_mcap_streaming();
#endif

  if (failures != 0) {
    std::cerr << failures << " session recorder test(s) failed\n";
    return 1;
  }
  std::cout << "runtime session recorder tests passed\n";
  return 0;
}
