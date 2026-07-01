// The single translation unit that compiles the vendored SQLite amalgamation and
// the SQLite-backed Catalog. sqlite3.h is included here only, so the dependency
// never leaks past this file.

#include <cavr/catalog/sqlite_catalog.hpp>

#include <sqlite3.h>

#include <exception>
#include <string>

namespace cavr::catalog {

namespace {

[[nodiscard]] std::string column_text(sqlite3_stmt* stmt, int col) {
  const unsigned char* text = sqlite3_column_text(stmt, col);
  return text ? reinterpret_cast<const char*>(text) : std::string{};
}

}  // namespace

struct SqliteCatalog::Impl {
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

SqliteCatalog::SqliteCatalog(const CatalogOpenOptions& options) : impl_(std::make_unique<Impl>()) {
  int flags = SQLITE_OPEN_READWRITE;
  if (options.create_if_missing) flags |= SQLITE_OPEN_CREATE;
  if (sqlite3_open_v2(options.path.c_str(), &impl_->db, flags, nullptr) == SQLITE_OK) {
    impl_->opened = true;
  } else {
    impl_->open_error = std::string("Failed to open catalog: ") +
                        (impl_->db ? sqlite3_errmsg(impl_->db) : "unknown error");
  }
}

SqliteCatalog::~SqliteCatalog() {
  if (impl_ && impl_->db) sqlite3_close(impl_->db);
}

CatalogStatus SqliteCatalog::initialize() {
  if (!impl_->opened) return CatalogStatus::failure(impl_->open_error);

  if (CatalogStatus s = impl_->exec(
          "CREATE TABLE IF NOT EXISTS catalog_meta (key TEXT PRIMARY KEY, value TEXT);"
          "CREATE TABLE IF NOT EXISTS sessions ("
          "  id TEXT PRIMARY KEY, name TEXT, file_path TEXT,"
          "  start_ns INTEGER, duration_ns INTEGER,"
          "  robot_model TEXT, camera_model TEXT, calibration_id TEXT,"
          "  file_size INTEGER, content_hash TEXT);"
          "CREATE TABLE IF NOT EXISTS tags ("
          "  session_id TEXT, tag TEXT, PRIMARY KEY (session_id, tag));"
          "CREATE TABLE IF NOT EXISTS annotations ("
          "  session_id TEXT, t_ns INTEGER, author TEXT, text TEXT);"
          "CREATE TABLE IF NOT EXISTS bookmarks ("
          "  session_id TEXT, t_ns INTEGER, label TEXT);"
          "CREATE TABLE IF NOT EXISTS validation_runs ("
          "  session_id TEXT, passed INTEGER, error_count INTEGER, warning_count INTEGER,"
          "  collisions_evaluated INTEGER, detail TEXT, created_ns INTEGER);");
      !s) {
    return s;
  }

  // Read the stored schema version, or stamp it on a fresh catalog.
  sqlite3_stmt* stmt = nullptr;
  sqlite3_prepare_v2(impl_->db, "SELECT value FROM catalog_meta WHERE key='schema_version';", -1,
                     &stmt, nullptr);
  bool has_version = false;
  if (stmt && sqlite3_step(stmt) == SQLITE_ROW) {
    try {
      impl_->version = std::stoi(column_text(stmt, 0));
      has_version = true;
    } catch (const std::exception&) {
      // A catalog_meta row that isn't a parseable integer is treated as absent
      // rather than propagating an exception out of this SQLite glue code.
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

CatalogStatus SqliteCatalog::upsert_session(const CatalogSession& session) {
  if (!impl_->opened) return CatalogStatus::failure(impl_->open_error);
  if (session.id.empty()) return CatalogStatus::failure("Session id must not be empty");

  sqlite3_stmt* stmt = nullptr;
  if (sqlite3_prepare_v2(impl_->db,
                         "INSERT OR REPLACE INTO sessions"
                         "(id, name, file_path, start_ns, duration_ns, robot_model,"
                         " camera_model, calibration_id, file_size, content_hash)"
                         " VALUES (?,?,?,?,?,?,?,?,?,?);",
                         -1, &stmt, nullptr) != SQLITE_OK) {
    return CatalogStatus::failure(sqlite3_errmsg(impl_->db));
  }

  sqlite3_bind_text(stmt, 1, session.id.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 2, session.name.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 3, session.file_path.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_int64(stmt, 4, session.start_ns);
  sqlite3_bind_int64(stmt, 5, session.duration_ns);
  sqlite3_bind_text(stmt, 6, session.robot_model.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 7, session.camera_model.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 8, session.calibration_id.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_int64(stmt, 9, static_cast<sqlite3_int64>(session.file_size));
  sqlite3_bind_text(stmt, 10, session.content_hash.c_str(), -1, SQLITE_TRANSIENT);

  const int rc = sqlite3_step(stmt);
  sqlite3_finalize(stmt);
  if (rc != SQLITE_DONE) return CatalogStatus::failure(sqlite3_errmsg(impl_->db));
  return CatalogStatus::success();
}

namespace {

[[nodiscard]] CatalogSession read_session_row(sqlite3_stmt* stmt) {
  CatalogSession s;
  s.id = column_text(stmt, 0);
  s.name = column_text(stmt, 1);
  s.file_path = column_text(stmt, 2);
  s.start_ns = sqlite3_column_int64(stmt, 3);
  s.duration_ns = sqlite3_column_int64(stmt, 4);
  s.robot_model = column_text(stmt, 5);
  s.camera_model = column_text(stmt, 6);
  s.calibration_id = column_text(stmt, 7);
  s.file_size = static_cast<std::uint64_t>(sqlite3_column_int64(stmt, 8));
  s.content_hash = column_text(stmt, 9);
  return s;
}

constexpr const char* kSelectColumns =
    "id, name, file_path, start_ns, duration_ns, robot_model, camera_model,"
    " calibration_id, file_size, content_hash";

}  // namespace

std::vector<CatalogSession> SqliteCatalog::list_sessions() const {
  std::vector<CatalogSession> out;
  if (!impl_->opened) return out;

  const std::string sql = std::string("SELECT ") + kSelectColumns + " FROM sessions ORDER BY id;";
  sqlite3_stmt* stmt = nullptr;
  if (sqlite3_prepare_v2(impl_->db, sql.c_str(), -1, &stmt, nullptr) != SQLITE_OK) return out;
  while (sqlite3_step(stmt) == SQLITE_ROW) out.push_back(read_session_row(stmt));
  sqlite3_finalize(stmt);
  return out;
}

std::optional<CatalogSession> SqliteCatalog::find_session(const SessionId& id) const {
  if (!impl_->opened) return std::nullopt;

  const std::string sql = std::string("SELECT ") + kSelectColumns + " FROM sessions WHERE id=?;";
  sqlite3_stmt* stmt = nullptr;
  if (sqlite3_prepare_v2(impl_->db, sql.c_str(), -1, &stmt, nullptr) != SQLITE_OK) return std::nullopt;
  sqlite3_bind_text(stmt, 1, id.c_str(), -1, SQLITE_TRANSIENT);

  std::optional<CatalogSession> result;
  if (sqlite3_step(stmt) == SQLITE_ROW) result = read_session_row(stmt);
  sqlite3_finalize(stmt);
  return result;
}

CatalogStatus SqliteCatalog::add_tag(const SessionId& id, const std::string& tag) {
  if (!impl_->opened) return CatalogStatus::failure(impl_->open_error);
  if (!find_session(id)) return CatalogStatus::failure("Unknown session: " + id);

  sqlite3_stmt* stmt = nullptr;
  if (sqlite3_prepare_v2(impl_->db, "INSERT OR IGNORE INTO tags(session_id, tag) VALUES (?,?);", -1,
                         &stmt, nullptr) != SQLITE_OK) {
    return CatalogStatus::failure(sqlite3_errmsg(impl_->db));
  }
  sqlite3_bind_text(stmt, 1, id.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 2, tag.c_str(), -1, SQLITE_TRANSIENT);
  const int rc = sqlite3_step(stmt);
  sqlite3_finalize(stmt);
  if (rc != SQLITE_DONE) return CatalogStatus::failure(sqlite3_errmsg(impl_->db));
  return CatalogStatus::success();
}

std::vector<std::string> SqliteCatalog::tags_for(const SessionId& id) const {
  std::vector<std::string> out;
  if (!impl_->opened) return out;

  sqlite3_stmt* stmt = nullptr;
  if (sqlite3_prepare_v2(impl_->db, "SELECT tag FROM tags WHERE session_id=? ORDER BY tag;", -1,
                         &stmt, nullptr) != SQLITE_OK) {
    return out;
  }
  sqlite3_bind_text(stmt, 1, id.c_str(), -1, SQLITE_TRANSIENT);
  while (sqlite3_step(stmt) == SQLITE_ROW) out.push_back(column_text(stmt, 0));
  sqlite3_finalize(stmt);
  return out;
}

CatalogStatus SqliteCatalog::add_annotation(const SessionId& id, const CatalogAnnotation& annotation) {
  if (!impl_->opened) return CatalogStatus::failure(impl_->open_error);
  if (!find_session(id)) return CatalogStatus::failure("Unknown session: " + id);

  sqlite3_stmt* stmt = nullptr;
  if (sqlite3_prepare_v2(impl_->db,
                         "INSERT INTO annotations(session_id, t_ns, author, text) VALUES (?,?,?,?);",
                         -1, &stmt, nullptr) != SQLITE_OK) {
    return CatalogStatus::failure(sqlite3_errmsg(impl_->db));
  }
  sqlite3_bind_text(stmt, 1, id.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_int64(stmt, 2, annotation.t_ns);
  sqlite3_bind_text(stmt, 3, annotation.author.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 4, annotation.text.c_str(), -1, SQLITE_TRANSIENT);
  const int rc = sqlite3_step(stmt);
  sqlite3_finalize(stmt);
  if (rc != SQLITE_DONE) return CatalogStatus::failure(sqlite3_errmsg(impl_->db));
  return CatalogStatus::success();
}

std::vector<CatalogAnnotation> SqliteCatalog::annotations_for(const SessionId& id) const {
  std::vector<CatalogAnnotation> out;
  if (!impl_->opened) return out;
  sqlite3_stmt* stmt = nullptr;
  if (sqlite3_prepare_v2(impl_->db,
                         "SELECT t_ns, author, text FROM annotations WHERE session_id=? ORDER BY t_ns;",
                         -1, &stmt, nullptr) != SQLITE_OK) {
    return out;
  }
  sqlite3_bind_text(stmt, 1, id.c_str(), -1, SQLITE_TRANSIENT);
  while (sqlite3_step(stmt) == SQLITE_ROW) {
    CatalogAnnotation a;
    a.t_ns = sqlite3_column_int64(stmt, 0);
    a.author = column_text(stmt, 1);
    a.text = column_text(stmt, 2);
    out.push_back(std::move(a));
  }
  sqlite3_finalize(stmt);
  return out;
}

CatalogStatus SqliteCatalog::add_bookmark(const SessionId& id, const CatalogBookmark& bookmark) {
  if (!impl_->opened) return CatalogStatus::failure(impl_->open_error);
  if (!find_session(id)) return CatalogStatus::failure("Unknown session: " + id);

  sqlite3_stmt* stmt = nullptr;
  if (sqlite3_prepare_v2(impl_->db, "INSERT INTO bookmarks(session_id, t_ns, label) VALUES (?,?,?);",
                         -1, &stmt, nullptr) != SQLITE_OK) {
    return CatalogStatus::failure(sqlite3_errmsg(impl_->db));
  }
  sqlite3_bind_text(stmt, 1, id.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_int64(stmt, 2, bookmark.t_ns);
  sqlite3_bind_text(stmt, 3, bookmark.label.c_str(), -1, SQLITE_TRANSIENT);
  const int rc = sqlite3_step(stmt);
  sqlite3_finalize(stmt);
  if (rc != SQLITE_DONE) return CatalogStatus::failure(sqlite3_errmsg(impl_->db));
  return CatalogStatus::success();
}

std::vector<CatalogBookmark> SqliteCatalog::bookmarks_for(const SessionId& id) const {
  std::vector<CatalogBookmark> out;
  if (!impl_->opened) return out;
  sqlite3_stmt* stmt = nullptr;
  if (sqlite3_prepare_v2(impl_->db,
                         "SELECT t_ns, label FROM bookmarks WHERE session_id=? ORDER BY t_ns;", -1,
                         &stmt, nullptr) != SQLITE_OK) {
    return out;
  }
  sqlite3_bind_text(stmt, 1, id.c_str(), -1, SQLITE_TRANSIENT);
  while (sqlite3_step(stmt) == SQLITE_ROW) {
    CatalogBookmark b;
    b.t_ns = sqlite3_column_int64(stmt, 0);
    b.label = column_text(stmt, 1);
    out.push_back(std::move(b));
  }
  sqlite3_finalize(stmt);
  return out;
}

CatalogStatus SqliteCatalog::add_validation_summary(const SessionId& id,
                                                    const ValidationSummary& summary) {
  if (!impl_->opened) return CatalogStatus::failure(impl_->open_error);
  if (!find_session(id)) return CatalogStatus::failure("Unknown session: " + id);

  sqlite3_stmt* stmt = nullptr;
  if (sqlite3_prepare_v2(impl_->db,
                         "INSERT INTO validation_runs(session_id, passed, error_count, warning_count,"
                         " collisions_evaluated, detail, created_ns) VALUES (?,?,?,?,?,?,?);",
                         -1, &stmt, nullptr) != SQLITE_OK) {
    return CatalogStatus::failure(sqlite3_errmsg(impl_->db));
  }
  sqlite3_bind_text(stmt, 1, id.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_int(stmt, 2, summary.passed ? 1 : 0);
  sqlite3_bind_int(stmt, 3, summary.error_count);
  sqlite3_bind_int(stmt, 4, summary.warning_count);
  sqlite3_bind_int(stmt, 5, summary.collisions_evaluated ? 1 : 0);
  sqlite3_bind_text(stmt, 6, summary.detail.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_int64(stmt, 7, summary.created_ns);
  const int rc = sqlite3_step(stmt);
  sqlite3_finalize(stmt);
  if (rc != SQLITE_DONE) return CatalogStatus::failure(sqlite3_errmsg(impl_->db));
  return CatalogStatus::success();
}

std::vector<ValidationSummary> SqliteCatalog::validation_summaries_for(const SessionId& id) const {
  std::vector<ValidationSummary> out;
  if (!impl_->opened) return out;
  sqlite3_stmt* stmt = nullptr;
  if (sqlite3_prepare_v2(impl_->db,
                         "SELECT passed, error_count, warning_count, collisions_evaluated, detail,"
                         " created_ns FROM validation_runs WHERE session_id=? ORDER BY created_ns;",
                         -1, &stmt, nullptr) != SQLITE_OK) {
    return out;
  }
  sqlite3_bind_text(stmt, 1, id.c_str(), -1, SQLITE_TRANSIENT);
  while (sqlite3_step(stmt) == SQLITE_ROW) {
    ValidationSummary v;
    v.passed = sqlite3_column_int(stmt, 0) != 0;
    v.error_count = sqlite3_column_int(stmt, 1);
    v.warning_count = sqlite3_column_int(stmt, 2);
    v.collisions_evaluated = sqlite3_column_int(stmt, 3) != 0;
    v.detail = column_text(stmt, 4);
    v.created_ns = sqlite3_column_int64(stmt, 5);
    out.push_back(std::move(v));
  }
  sqlite3_finalize(stmt);
  return out;
}

int SqliteCatalog::schema_version() const { return impl_->version; }

}  // namespace cavr::catalog
