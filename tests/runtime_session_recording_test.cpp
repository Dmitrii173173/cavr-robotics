// End-to-end test of the SessionLog <-> recording bridge: run a real session
// against the mock controller, then persist and replay it through the neutral
// recording interfaces — via the JSON reference backend and, when enabled, via
// the MCAP backend. The bridge must be backend-agnostic, so both paths assert the
// same round-trip guarantees.

#include <cavr/adapters/mock_robot/mock_controller.hpp>
#include <cavr/record/json_recording.hpp>
#include <cavr/record/reader.hpp>
#include <cavr/record/writer.hpp>
#include <cavr/runtime/demo_plan.hpp>
#include <cavr/runtime/record_session.hpp>
#include <cavr/runtime/session_manager.hpp>

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
namespace machine = cavr::machine;

std::filesystem::path temp_path(std::string_view name) {
  return std::filesystem::temp_directory_path() / name;
}

// Drives the mock controller to completion and returns the recorded SessionLog.
runtime::SessionLog run_demo_session() {
  mock::MockController controller;
  runtime::SessionManager manager;
  (void)manager.connect(controller, {"mock", "mock"});
  (void)manager.discover_profile();
  manager.set_plan(runtime::make_demo_plan());
  (void)manager.validate();
  const bool started = manager.execute("session_bridge_test");
  check(started, "demo session starts executing");

  std::int64_t now_ns = 1'000'000'000;
  int ticks = 0;
  while (manager.phase() == runtime::SessionPhase::Executing && ticks < 5000) {
    manager.tick(cavr::core::Timestamp::from_nanoseconds(now_ns));
    now_ns += 20'000'000;  // 50 Hz
    ++ticks;
  }
  return manager.log();
}

// Asserts a reconstructed log matches the original across the bridge.
void check_round_trip(const runtime::SessionLog& original, const runtime::SessionRecordingResult& loaded,
                      std::string_view backend) {
  const std::string ctx = std::string(backend) + ": ";
  check(loaded.ok(), (ctx + "session reads back without error").c_str());
  check(loaded.log.session_id == original.session_id, (ctx + "session id round-trips").c_str());
  check(loaded.log.frame_count() == original.frame_count(), (ctx + "frame count round-trips").c_str());
  check(loaded.log.profile.dof() == original.profile.dof(), (ctx + "profile round-trips").c_str());
  check(loaded.log.timeline.events.size() == original.timeline.events.size(),
        (ctx + "controller events round-trip").c_str());
  check(loaded.log.timeline.steps.size() == original.timeline.steps.size(),
        (ctx + "plan steps round-trip").c_str());
  check(loaded.log.started == original.started && loaded.log.ended == original.ended,
        (ctx + "session span round-trips").c_str());

  // The reconstructed log must be replayable.
  runtime::ReplayCursor cursor(loaded.log);
  check(cursor.duration_s() > 0.0, (ctx + "replay has positive duration").c_str());
  const auto* mid = cursor.frame_at(cursor.duration_s() * 0.5);
  check(mid != nullptr && mid->joint_positions.size() == 6,
        (ctx + "replay yields a six-joint frame").c_str());
}

void test_json_backend() {
  const runtime::SessionLog original = run_demo_session();
  check(original.frame_count() > 10, "json: demo session recorded telemetry");

  const auto path = temp_path("cavr_session_bridge.json");
  std::filesystem::remove(path);

  record::JsonRecordingWriter writer(path);
  check(static_cast<bool>(runtime::write_session(original, writer)), "json: session writes through the bridge");

  const auto loaded = record::load_recording(path);
  check(static_cast<bool>(loaded.status), "json: recording file loads");
  record::JsonRecordingReader reader(loaded.recording);

  check_round_trip(original, runtime::read_session(reader), "json");

  std::filesystem::remove(path);
}

#ifdef CAVR_WITH_MCAP
void test_mcap_backend() {
  namespace storage_mcap = cavr::storage_mcap;
  const runtime::SessionLog original = run_demo_session();

  const auto path = temp_path("cavr_session_bridge.mcap");
  std::filesystem::remove(path);

  storage_mcap::McapRecordingWriter writer(path);
  check(static_cast<bool>(runtime::write_session(original, writer)), "mcap: session writes through the bridge");

  const auto loaded = storage_mcap::load_recording(path);
  check(static_cast<bool>(loaded.status), "mcap: .mcap file loads");
  storage_mcap::McapRecordingReader reader(loaded.recording);

  check_round_trip(original, runtime::read_session(reader), "mcap");

  std::filesystem::remove(path);
}
#endif

}  // namespace

int main() {
  test_json_backend();
#ifdef CAVR_WITH_MCAP
  test_mcap_backend();
#endif

  if (failures != 0) {
    std::cerr << failures << " session recording test(s) failed\n";
    return 1;
  }
  std::cout << "runtime session recording tests passed\n";
  return 0;
}
