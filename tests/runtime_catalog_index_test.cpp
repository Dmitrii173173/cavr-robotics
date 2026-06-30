// Closes the loop: record a real robot + camera session, then index that recording
// into the catalog by scanning it back. Proves the storage boundary — the heavy
// data lives in the recording, the catalog holds only reconstructible metadata
// that points at it — and that indexing is backend-agnostic (MCAP when built, else
// the JSON reference backend) and engine-agnostic (SQLite when built, else
// in-memory).

#include <cavr/adapters/mock_camera/mock_camera.hpp>
#include <cavr/adapters/mock_robot/mock_controller.hpp>
#include <cavr/catalog/in_memory_catalog.hpp>
#include <cavr/record/json_recording.hpp>
#include <cavr/runtime/catalog_index.hpp>
#include <cavr/runtime/demo_plan.hpp>
#include <cavr/runtime/session_manager.hpp>
#include <cavr/runtime/session_recorder.hpp>
#include <cavr/validation/trajectory_validator.hpp>

#ifdef CAVR_WITH_MCAP
#include <cavr/storage_mcap/mcap_recording.hpp>
#endif
#ifdef CAVR_WITH_SQLITE
#include <cavr/catalog/sqlite_catalog.hpp>
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

namespace runtime = cavr::runtime;
namespace record = cavr::record;
namespace catalog = cavr::catalog;
namespace mock = cavr::adapters::mock_robot;
namespace mock_cam = cavr::adapters::mock_camera;

std::filesystem::path temp_path(std::string_view name) {
  return std::filesystem::temp_directory_path() / name;
}

runtime::SessionLog run_recorded(runtime::SessionRecorder& recorder) {
  mock::MockController controller;
  mock_cam::MockCamera camera(8, 8, "weld_cam");
  runtime::SessionManager manager;
  manager.attach_recorder(recorder);
  manager.attach_camera(camera);

  (void)manager.connect(controller, {"mock", "mock"});
  (void)manager.discover_profile();
  manager.set_plan(runtime::make_demo_plan());
  (void)manager.validate();
  check(manager.execute("catalog_index_session"), "session starts");

  std::int64_t now_ns = 1'000'000'000;
  int ticks = 0;
  while (manager.phase() == runtime::SessionPhase::Executing && ticks < 5000) {
    manager.tick(cavr::core::Timestamp::from_nanoseconds(now_ns));
    now_ns += 20'000'000;
    ++ticks;
  }
  check(static_cast<bool>(recorder.finish(manager.log())), "recorder finalizes");
  return manager.log();
}

// Builds a catalog row by scanning the recording at `path` and checks it against
// the session that produced it.
catalog::CatalogSession index_and_check(const record::RecordingReader& reader,
                                        const std::filesystem::path& path,
                                        const runtime::SessionLog& log) {
  const auto fingerprint = catalog::file_fingerprint(path.string());
  check(fingerprint.ok && fingerprint.size > 0, "recording fingerprint computed");

  const catalog::CatalogSession entry =
      runtime::index_recording(reader, path.string(), fingerprint.size, fingerprint.hash);

  check(entry.id == log.session_id, "indexed id matches the session");
  check(entry.robot_model == "Yaskawa Motoman GP25", "indexed robot model is correct");
  check(entry.duration_ns > 0, "indexed duration is positive");
  check(entry.camera_model.find("weld_cam") != std::string::npos, "indexed camera model captured");
  check(entry.file_size == fingerprint.size, "indexed file size matches fingerprint");
  check(entry.content_hash == fingerprint.hash, "indexed content hash matches fingerprint");
  return entry;
}

void store_and_check(catalog::Catalog& cat, const catalog::CatalogSession& entry,
                     const catalog::ValidationSummary& validation, std::string_view engine) {
  const std::string ctx = std::string(engine) + ": ";
  check(static_cast<bool>(cat.initialize()), (ctx + "catalog initializes").c_str());
  check(static_cast<bool>(cat.upsert_session(entry)), (ctx + "session is catalogued").c_str());
  check(static_cast<bool>(cat.add_tag(entry.id, "welding")), (ctx + "session tagged").c_str());
  check(static_cast<bool>(cat.add_bookmark(entry.id, {entry.start_ns, "session start"})),
        (ctx + "bookmark added").c_str());
  check(static_cast<bool>(cat.add_validation_summary(entry.id, validation)),
        (ctx + "validation summary catalogued").c_str());

  const auto found = cat.find_session(entry.id);
  check(found && found->robot_model == entry.robot_model && found->file_size == entry.file_size,
        (ctx + "catalogued session is retrievable").c_str());
  check(cat.tags_for(entry.id).size() == 1, (ctx + "tag retrievable").c_str());
  check(cat.bookmarks_for(entry.id).size() == 1, (ctx + "bookmark retrievable").c_str());
  const auto summaries = cat.validation_summaries_for(entry.id);
  check(summaries.size() == 1 && summaries.front().passed && !summaries.front().collisions_evaluated,
        (ctx + "validation summary retrievable, collisions honestly not evaluated").c_str());
}

void test_index_into_catalog() {
#ifdef CAVR_WITH_MCAP
  namespace storage_mcap = cavr::storage_mcap;
  const auto path = temp_path("cavr_catalog_index.mcap");
  std::filesystem::remove(path);
  storage_mcap::McapRecordingWriter writer(path, /*streaming=*/true);
  runtime::SessionRecorder recorder(writer);
  const runtime::SessionLog log = run_recorded(recorder);

  const auto loaded = storage_mcap::load_recording(path);
  check(static_cast<bool>(loaded.status), "mcap recording loads for indexing");
  storage_mcap::McapRecordingReader reader(loaded.recording);
#else
  const auto path = temp_path("cavr_catalog_index.json");
  std::filesystem::remove(path);
  record::JsonRecordingWriter writer(path);
  runtime::SessionRecorder recorder(writer);
  const runtime::SessionLog log = run_recorded(recorder);

  const auto loaded = record::load_recording(path);
  check(static_cast<bool>(loaded.status), "json recording loads for indexing");
  record::JsonRecordingReader reader(loaded.recording);
#endif

  const catalog::CatalogSession entry = index_and_check(reader, path, log);

  // A real validation run, summarized for the catalog.
  namespace validation = cavr::validation;
  const auto report = validation::validate_task(log.profile, runtime::make_demo_plan().to_motion_task());
  const catalog::ValidationSummary summary = runtime::to_validation_summary(report, log.started.nanoseconds());
  check(summary.passed, "demo session validates clean");

  catalog::InMemoryCatalog in_memory;
  store_and_check(in_memory, entry, summary, "in-memory");

#ifdef CAVR_WITH_SQLITE
  const auto db = temp_path("cavr_catalog_index.db");
  std::filesystem::remove(db);
  catalog::SqliteCatalog sqlite({db.string(), true});
  store_and_check(sqlite, entry, summary, "sqlite");
  std::filesystem::remove(db);
#endif

  std::filesystem::remove(path);
}

}  // namespace

int main() {
  test_index_into_catalog();

  if (failures != 0) {
    std::cerr << failures << " catalog index test(s) failed\n";
    return 1;
  }
  std::cout << "runtime catalog index tests passed\n";
  return 0;
}
