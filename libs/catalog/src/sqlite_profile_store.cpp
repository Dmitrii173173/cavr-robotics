// SQLite-backed ProfileStore (the robot registry). Compiled in the catalog target
// alongside sqlite_catalog.cpp, so the vendored sqlite3 amalgamation is built once.
// sqlite3.h is included here only, so the dependency never leaks past this file.

#include <cavr/catalog/sqlite_profile_store.hpp>

#include <cavr/machine/profile_io.hpp>

#include <sqlite3.h>

#include <exception>
#include <string>

namespace cavr::catalog {

namespace {

[[nodiscard]] std::string column_text(sqlite3_stmt* stmt, int col) {
  const unsigned char* text = sqlite3_column_text(stmt, col);
  return text ? reinterpret_cast<const char*>(text) : std::string{};
}

constexpr const char* kSelectColumns =
    "id, display_name, adapter, transport, endpoint, profile_json, updated_ns";

[[nodiscard]] StoredRobot read_robot_row(sqlite3_stmt* stmt) {
  StoredRobot r;
  r.id = column_text(stmt, 0);
  r.display_name = column_text(stmt, 1);
  r.adapter = column_text(stmt, 2);
  r.transport = column_text(stmt, 3);
  r.endpoint = column_text(stmt, 4);
  r.profile = machine::parse_profile(column_text(stmt, 5)).profile;
  r.updated_ns = sqlite3_column_int64(stmt, 6);
  return r;
}

}  // namespace

struct SqliteProfileStore::Impl {
  sqlite3* db{nullptr};
  bool opened{false};
  std::string open_error;
  int version{0};

  [[nodiscard]] CatalogStatus exec(const char* sql) {
    char* error = nullptr;
    if (sqlite3_exec(db, sql, nullptr, nullptr, &error) != SQLITE_OK) {
      CatalogStatus status = CatalogStatus::failure(error ? error : "SQL error");
      sqlite3_free(error);
      return status;
    }
    return CatalogStatus::success();
  }
};

SqliteProfileStore::SqliteProfileStore(const CatalogOpenOptions& options)
    : impl_(std::make_unique<Impl>()) {
  int flags = SQLITE_OPEN_READWRITE;
  if (options.create_if_missing) flags |= SQLITE_OPEN_CREATE;
  if (sqlite3_open_v2(options.path.c_str(), &impl_->db, flags, nullptr) == SQLITE_OK) {
    impl_->opened = true;
  } else {
    impl_->open_error = std::string("Failed to open robot registry: ") +
                        (impl_->db ? sqlite3_errmsg(impl_->db) : "unknown error");
  }
}

SqliteProfileStore::~SqliteProfileStore() {
  if (impl_ && impl_->db) sqlite3_close(impl_->db);
}

CatalogStatus SqliteProfileStore::initialize() {
  if (!impl_->opened) return CatalogStatus::failure(impl_->open_error);

  // The robots table is additive; the schema-version row (shared with the session
  // catalog when they live in one file) gates cross-version compatibility.
  if (CatalogStatus s = impl_->exec(
          "CREATE TABLE IF NOT EXISTS catalog_meta (key TEXT PRIMARY KEY, value TEXT);"
          "CREATE TABLE IF NOT EXISTS robots ("
          "  id TEXT PRIMARY KEY, display_name TEXT, adapter TEXT, transport TEXT,"
          "  endpoint TEXT, profile_json TEXT, updated_ns INTEGER);");
      !s) {
    return s;
  }

  sqlite3_stmt* stmt = nullptr;
  sqlite3_prepare_v2(impl_->db, "SELECT value FROM catalog_meta WHERE key='schema_version';", -1,
                     &stmt, nullptr);
  bool has_version = false;
  if (stmt && sqlite3_step(stmt) == SQLITE_ROW) {
    try {
      impl_->version = std::stoi(column_text(stmt, 0));
      has_version = true;
    } catch (const std::exception&) {
      has_version = false;
    }
  }
  sqlite3_finalize(stmt);

  if (!has_version) {
    impl_->version = kCatalogSchemaVersion;
    if (CatalogStatus s = impl_->exec(
            "INSERT INTO catalog_meta(key, value) VALUES('schema_version', '1');");
        !s) {
      return s;
    }
  } else if (impl_->version > kCatalogSchemaVersion) {
    return CatalogStatus::failure("Unsupported catalog schema version " +
                                  std::to_string(impl_->version) + " (this build supports " +
                                  std::to_string(kCatalogSchemaVersion) + ")");
  }
  return CatalogStatus::success();
}

CatalogStatus SqliteProfileStore::upsert_robot(const StoredRobot& robot) {
  if (!impl_->opened) return CatalogStatus::failure(impl_->open_error);
  if (robot.id.empty()) return CatalogStatus::failure("Robot id must not be empty");

  const std::string profile_json = machine::export_profile_string(robot.profile);

  sqlite3_stmt* stmt = nullptr;
  if (sqlite3_prepare_v2(impl_->db,
                         "INSERT OR REPLACE INTO robots"
                         "(id, display_name, adapter, transport, endpoint, profile_json, updated_ns)"
                         " VALUES (?,?,?,?,?,?,?);",
                         -1, &stmt, nullptr) != SQLITE_OK) {
    return CatalogStatus::failure(sqlite3_errmsg(impl_->db));
  }
  sqlite3_bind_text(stmt, 1, robot.id.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 2, robot.display_name.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 3, robot.adapter.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 4, robot.transport.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 5, robot.endpoint.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 6, profile_json.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_int64(stmt, 7, robot.updated_ns);

  const int rc = sqlite3_step(stmt);
  sqlite3_finalize(stmt);
  if (rc != SQLITE_DONE) return CatalogStatus::failure(sqlite3_errmsg(impl_->db));
  return CatalogStatus::success();
}

std::vector<StoredRobot> SqliteProfileStore::list_robots() const {
  std::vector<StoredRobot> out;
  if (!impl_->opened) return out;

  const std::string sql = std::string("SELECT ") + kSelectColumns + " FROM robots ORDER BY id;";
  sqlite3_stmt* stmt = nullptr;
  if (sqlite3_prepare_v2(impl_->db, sql.c_str(), -1, &stmt, nullptr) != SQLITE_OK) return out;
  while (sqlite3_step(stmt) == SQLITE_ROW) out.push_back(read_robot_row(stmt));
  sqlite3_finalize(stmt);
  return out;
}

std::optional<StoredRobot> SqliteProfileStore::find_robot(const std::string& id) const {
  if (!impl_->opened) return std::nullopt;

  const std::string sql = std::string("SELECT ") + kSelectColumns + " FROM robots WHERE id=?;";
  sqlite3_stmt* stmt = nullptr;
  if (sqlite3_prepare_v2(impl_->db, sql.c_str(), -1, &stmt, nullptr) != SQLITE_OK) return std::nullopt;
  sqlite3_bind_text(stmt, 1, id.c_str(), -1, SQLITE_TRANSIENT);

  std::optional<StoredRobot> result;
  if (sqlite3_step(stmt) == SQLITE_ROW) result = read_robot_row(stmt);
  sqlite3_finalize(stmt);
  return result;
}

CatalogStatus SqliteProfileStore::delete_robot(const std::string& id) {
  if (!impl_->opened) return CatalogStatus::failure(impl_->open_error);

  sqlite3_stmt* stmt = nullptr;
  if (sqlite3_prepare_v2(impl_->db, "DELETE FROM robots WHERE id=?;", -1, &stmt, nullptr) !=
      SQLITE_OK) {
    return CatalogStatus::failure(sqlite3_errmsg(impl_->db));
  }
  sqlite3_bind_text(stmt, 1, id.c_str(), -1, SQLITE_TRANSIENT);
  const int rc = sqlite3_step(stmt);
  sqlite3_finalize(stmt);
  if (rc != SQLITE_DONE) return CatalogStatus::failure(sqlite3_errmsg(impl_->db));
  return CatalogStatus::success();
}

int SqliteProfileStore::schema_version() const { return impl_->version; }

}  // namespace cavr::catalog
