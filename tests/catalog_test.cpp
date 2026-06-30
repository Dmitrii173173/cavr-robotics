// Catalog contract: the same semantics must hold for every engine. The in-memory
// reference and the SQLite backend are both driven through the Catalog interface
// by one shared test body, plus a few SQLite-specific checks (persistence across
// reopen, rejection of a future schema version).

#include <cavr/catalog/in_memory_catalog.hpp>

#ifdef CAVR_WITH_SQLITE
#include <cavr/catalog/sqlite_catalog.hpp>
#include <sqlite3.h>
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

namespace catalog = cavr::catalog;

catalog::CatalogSession make_session(std::string id, std::string name) {
  catalog::CatalogSession s;
  s.id = std::move(id);
  s.name = std::move(name);
  s.file_path = "/recordings/" + s.id + ".mcap";
  s.start_ns = 1'000'000'000;
  s.duration_ns = 2'500'000'000;
  s.robot_model = "Yaskawa Motoman GP25";
  s.camera_model = "weld_cam (mono8)";
  s.file_size = 4096;
  s.content_hash = "deadbeefdeadbeef";
  return s;
}

// The engine-independent contract, run against any Catalog.
void run_contract(catalog::Catalog& cat, std::string_view engine) {
  const std::string ctx = std::string(engine) + ": ";

  check(static_cast<bool>(cat.initialize()), (ctx + "initialize succeeds").c_str());
  check(cat.schema_version() == catalog::kCatalogSchemaVersion,
        (ctx + "schema version is current").c_str());

  // insert + read back
  check(static_cast<bool>(cat.upsert_session(make_session("u1", "demo"))),
        (ctx + "insert session").c_str());
  const auto got = cat.find_session("u1");
  check(got.has_value(), (ctx + "session reads back").c_str());
  check(got && got->robot_model == "Yaskawa Motoman GP25" && got->file_size == 4096 &&
            got->start_ns == 1'000'000'000,
        (ctx + "session fields round-trip").c_str());

  // missing session is a controlled empty result
  check(!cat.find_session("nope").has_value(), (ctx + "missing session returns empty").c_str());

  // update existing (upsert replaces, does not duplicate)
  auto updated = make_session("u1", "renamed");
  updated.duration_ns = 9'000'000'000;
  check(static_cast<bool>(cat.upsert_session(updated)), (ctx + "update session").c_str());
  const auto after = cat.find_session("u1");
  check(after && after->name == "renamed" && after->duration_ns == 9'000'000'000,
        (ctx + "update is reflected").c_str());
  check(cat.list_sessions().size() == 1, (ctx + "update did not duplicate").c_str());

  // tags
  check(static_cast<bool>(cat.add_tag("u1", "welding")), (ctx + "add tag").c_str());
  check(static_cast<bool>(cat.add_tag("u1", "good")), (ctx + "add second tag").c_str());
  check(static_cast<bool>(cat.add_tag("u1", "welding")), (ctx + "duplicate tag is ignored").c_str());
  check(cat.tags_for("u1").size() == 2, (ctx + "two distinct tags stored").c_str());
  check(!cat.add_tag("missing", "x").ok, (ctx + "tagging unknown session fails").c_str());

  // annotations (time-anchored and session-level)
  check(static_cast<bool>(cat.add_annotation("u1", {1500, "operator", "weld looked clean"})),
        (ctx + "add time-anchored annotation").c_str());
  check(static_cast<bool>(cat.add_annotation("u1", {-1, "qa", "session note"})),
        (ctx + "add session-level annotation").c_str());
  check(cat.annotations_for("u1").size() == 2, (ctx + "annotations stored").c_str());
  check(!cat.add_annotation("missing", {}).ok, (ctx + "annotation on unknown session fails").c_str());

  // bookmarks
  check(static_cast<bool>(cat.add_bookmark("u1", {2000, "arc start"})), (ctx + "add bookmark").c_str());
  const auto bookmarks = cat.bookmarks_for("u1");
  check(bookmarks.size() == 1 && bookmarks.front().label == "arc start",
        (ctx + "bookmark stored").c_str());
  check(!cat.add_bookmark("missing", {}).ok, (ctx + "bookmark on unknown session fails").c_str());

  // validation run summaries
  catalog::ValidationSummary summary;
  summary.passed = true;
  summary.warning_count = 2;
  summary.collisions_evaluated = false;
  summary.detail = "speed warning on axis T";
  summary.created_ns = 12345;
  check(static_cast<bool>(cat.add_validation_summary("u1", summary)),
        (ctx + "add validation summary").c_str());
  const auto summaries = cat.validation_summaries_for("u1");
  check(summaries.size() == 1 && summaries.front().warning_count == 2 &&
            !summaries.front().collisions_evaluated && summaries.front().detail == "speed warning on axis T",
        (ctx + "validation summary round-trips").c_str());
  check(!cat.add_validation_summary("missing", {}).ok,
        (ctx + "validation summary on unknown session fails").c_str());

  // listing is ordered by id
  check(static_cast<bool>(cat.upsert_session(make_session("u0", "earlier"))),
        (ctx + "insert second session").c_str());
  const auto all = cat.list_sessions();
  check(all.size() == 2 && all.front().id == "u0", (ctx + "sessions list ordered by id").c_str());
}

void test_in_memory() {
  catalog::InMemoryCatalog cat;
  run_contract(cat, "in-memory");
}

#ifdef CAVR_WITH_SQLITE
std::filesystem::path temp_db(std::string_view name) {
  return std::filesystem::temp_directory_path() / name;
}

void test_sqlite_contract() {
  const auto path = temp_db("cavr_catalog_contract.db");
  std::filesystem::remove(path);
  catalog::SqliteCatalog cat({path.string(), true});
  run_contract(cat, "sqlite");
  std::filesystem::remove(path);
}

// Data must survive closing and reopening the database file.
void test_sqlite_persistence() {
  const auto path = temp_db("cavr_catalog_persist.db");
  std::filesystem::remove(path);
  {
    catalog::SqliteCatalog cat({path.string(), true});
    check(static_cast<bool>(cat.initialize()), "sqlite: init for persistence");
    check(static_cast<bool>(cat.upsert_session(make_session("p1", "persisted"))),
          "sqlite: insert before close");
    check(static_cast<bool>(cat.add_tag("p1", "kept")), "sqlite: tag before close");
  }
  {
    catalog::SqliteCatalog cat({path.string(), true});
    check(static_cast<bool>(cat.initialize()), "sqlite: reopen existing catalog");
    const auto got = cat.find_session("p1");
    check(got && got->name == "persisted", "sqlite: session survived reopen");
    check(cat.tags_for("p1").size() == 1, "sqlite: tag survived reopen");
  }
  std::filesystem::remove(path);
}

// A catalog written by a newer schema version must be rejected, not mis-read.
void test_sqlite_future_version_rejected() {
  const auto path = temp_db("cavr_catalog_future.db");
  std::filesystem::remove(path);

  sqlite3* raw = nullptr;
  sqlite3_open(path.string().c_str(), &raw);
  sqlite3_exec(raw, "CREATE TABLE catalog_meta(key TEXT PRIMARY KEY, value TEXT);"
                    "INSERT INTO catalog_meta VALUES('schema_version','999');",
               nullptr, nullptr, nullptr);
  sqlite3_close(raw);

  catalog::SqliteCatalog cat({path.string(), true});
  const auto status = cat.initialize();
  check(!status.ok, "sqlite: future schema version is rejected");
  check(status.error.find("version") != std::string::npos, "sqlite: rejection names the version");

  std::filesystem::remove(path);
}
#endif

}  // namespace

int main() {
  test_in_memory();
#ifdef CAVR_WITH_SQLITE
  test_sqlite_contract();
  test_sqlite_persistence();
  test_sqlite_future_version_rejected();
#endif

  if (failures != 0) {
    std::cerr << failures << " catalog test(s) failed\n";
    return 1;
  }
  std::cout << "catalog tests passed\n";
  return 0;
}
