// SQLite-backed ProgramStore (the job registry). Compiled in the catalog target
// alongside the other SQLite backends; sqlite3.h is included here only.

#include <cavr/catalog/sqlite_program_store.hpp>

#include <cavr/machine/motion_io.hpp>

#include <sqlite3.h>

#include <exception>
#include <string>

namespace cavr::catalog {

namespace {

[[nodiscard]] std::string column_text(sqlite3_stmt* stmt, int col) {
  const unsigned char* text = sqlite3_column_text(stmt, col);
  return text ? reinterpret_cast<const char*>(text) : std::string{};
}

constexpr const char* kSelectColumns = "id, name, robot_id, task_json, updated_ns";

[[nodiscard]] StoredProgram read_program_row(sqlite3_stmt* stmt) {
  StoredProgram p;
  p.id = column_text(stmt, 0);
  p.name = column_text(stmt, 1);
  p.robot_id = column_text(stmt, 2);
  p.task = machine::parse_task(column_text(stmt, 3));
  p.updated_ns = sqlite3_column_int64(stmt, 4);
  return p;
}

}  // namespace

struct SqliteProgramStore::Impl {
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

SqliteProgramStore::SqliteProgramStore(const CatalogOpenOptions& options)
    : impl_(std::make_unique<Impl>()) {
  int flags = SQLITE_OPEN_READWRITE;
  if (options.create_if_missing) flags |= SQLITE_OPEN_CREATE;
  if (sqlite3_open_v2(options.path.c_str(), &impl_->db, flags, nullptr) == SQLITE_OK) {
    impl_->opened = true;
  } else {
    impl_->open_error = std::string("Failed to open program registry: ") +
                        (impl_->db ? sqlite3_errmsg(impl_->db) : "unknown error");
  }
}

SqliteProgramStore::~SqliteProgramStore() {
  if (impl_ && impl_->db) sqlite3_close(impl_->db);
}

CatalogStatus SqliteProgramStore::initialize() {
  if (!impl_->opened) return CatalogStatus::failure(impl_->open_error);

  if (CatalogStatus s = impl_->exec(
          "CREATE TABLE IF NOT EXISTS catalog_meta (key TEXT PRIMARY KEY, value TEXT);"
          "CREATE TABLE IF NOT EXISTS programs ("
          "  id TEXT PRIMARY KEY, name TEXT, robot_id TEXT, task_json TEXT, updated_ns INTEGER);");
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

CatalogStatus SqliteProgramStore::upsert_program(const StoredProgram& program) {
  if (!impl_->opened) return CatalogStatus::failure(impl_->open_error);
  if (program.id.empty()) return CatalogStatus::failure("Program id must not be empty");

  const std::string task_json = machine::export_task_string(program.task);

  sqlite3_stmt* stmt = nullptr;
  if (sqlite3_prepare_v2(impl_->db,
                         "INSERT OR REPLACE INTO programs"
                         "(id, name, robot_id, task_json, updated_ns) VALUES (?,?,?,?,?);",
                         -1, &stmt, nullptr) != SQLITE_OK) {
    return CatalogStatus::failure(sqlite3_errmsg(impl_->db));
  }
  sqlite3_bind_text(stmt, 1, program.id.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 2, program.name.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 3, program.robot_id.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 4, task_json.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_int64(stmt, 5, program.updated_ns);

  const int rc = sqlite3_step(stmt);
  sqlite3_finalize(stmt);
  if (rc != SQLITE_DONE) return CatalogStatus::failure(sqlite3_errmsg(impl_->db));
  return CatalogStatus::success();
}

std::vector<StoredProgram> SqliteProgramStore::list_programs() const {
  std::vector<StoredProgram> out;
  if (!impl_->opened) return out;

  const std::string sql = std::string("SELECT ") + kSelectColumns + " FROM programs ORDER BY id;";
  sqlite3_stmt* stmt = nullptr;
  if (sqlite3_prepare_v2(impl_->db, sql.c_str(), -1, &stmt, nullptr) != SQLITE_OK) return out;
  while (sqlite3_step(stmt) == SQLITE_ROW) out.push_back(read_program_row(stmt));
  sqlite3_finalize(stmt);
  return out;
}

std::optional<StoredProgram> SqliteProgramStore::find_program(const std::string& id) const {
  if (!impl_->opened) return std::nullopt;

  const std::string sql = std::string("SELECT ") + kSelectColumns + " FROM programs WHERE id=?;";
  sqlite3_stmt* stmt = nullptr;
  if (sqlite3_prepare_v2(impl_->db, sql.c_str(), -1, &stmt, nullptr) != SQLITE_OK) return std::nullopt;
  sqlite3_bind_text(stmt, 1, id.c_str(), -1, SQLITE_TRANSIENT);

  std::optional<StoredProgram> result;
  if (sqlite3_step(stmt) == SQLITE_ROW) result = read_program_row(stmt);
  sqlite3_finalize(stmt);
  return result;
}

CatalogStatus SqliteProgramStore::delete_program(const std::string& id) {
  if (!impl_->opened) return CatalogStatus::failure(impl_->open_error);

  sqlite3_stmt* stmt = nullptr;
  if (sqlite3_prepare_v2(impl_->db, "DELETE FROM programs WHERE id=?;", -1, &stmt, nullptr) !=
      SQLITE_OK) {
    return CatalogStatus::failure(sqlite3_errmsg(impl_->db));
  }
  sqlite3_bind_text(stmt, 1, id.c_str(), -1, SQLITE_TRANSIENT);
  const int rc = sqlite3_step(stmt);
  sqlite3_finalize(stmt);
  if (rc != SQLITE_DONE) return CatalogStatus::failure(sqlite3_errmsg(impl_->db));
  return CatalogStatus::success();
}

int SqliteProgramStore::schema_version() const { return impl_->version; }

}  // namespace cavr::catalog
