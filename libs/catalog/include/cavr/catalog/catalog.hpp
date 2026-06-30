#pragma once

// Local session catalog: a small index of recorded sessions for browsing and
// search. It stores only lightweight, reconstructible metadata — never telemetry,
// images or point clouds. The authoritative data lives in the recordings (MCAP)
// the catalog points at; deleting or rebuilding the catalog never touches it, and
// the catalog can be rebuilt by re-scanning the recording files.
//
// The storage engine is hidden behind the Catalog interface: an in-memory
// reference implementation and a SQLite-backed one implement it identically.

#include <cstdint>
#include <fstream>
#include <optional>
#include <string>
#include <vector>

namespace cavr::catalog {

// Bump when the on-disk schema changes; a catalog from a newer schema is rejected.
inline constexpr int kCatalogSchemaVersion = 1;

using SessionId = std::string;  // the recording's session id / UUID

// One row of reconstructible session metadata. Every field can be recovered by
// re-scanning the recording file, so the catalog is a cache, not a source of truth.
struct CatalogSession final {
  SessionId id;
  std::string name;
  std::string file_path;        // path to the recording (.mcap / .json)
  std::int64_t start_ns{0};     // session start, ns since epoch
  std::int64_t duration_ns{0};
  std::string robot_model;
  std::string camera_model;
  std::string calibration_id;
  std::uint64_t file_size{0};
  std::string content_hash;     // fingerprint of the recording bytes
};

// A free-text note on a session, optionally anchored to an instant (t_ns < 0 means
// the note applies to the whole session).
struct CatalogAnnotation final {
  std::int64_t t_ns{-1};
  std::string author;
  std::string text;
};

// A named instant for quick navigation during replay.
struct CatalogBookmark final {
  std::int64_t t_ns{0};
  std::string label;
};

// The outcome of one pre-execution validation run, summarized for browsing.
struct ValidationSummary final {
  bool passed{true};
  int error_count{0};
  int warning_count{0};
  bool collisions_evaluated{false};
  std::string detail;
  std::int64_t created_ns{0};
};

struct CatalogOpenOptions final {
  std::string path;                 // catalog database file (engine-specific)
  bool create_if_missing{true};
};

// Boolean outcome plus a human-readable error, matching the project's other I/O.
struct CatalogStatus final {
  bool ok{true};
  std::string error;

  [[nodiscard]] explicit operator bool() const noexcept { return ok; }
  [[nodiscard]] static CatalogStatus success() { return {}; }
  [[nodiscard]] static CatalogStatus failure(std::string message) {
    return CatalogStatus{false, std::move(message)};
  }
};

// Engine-neutral catalog. Implementations persist (or not) however they like.
class Catalog {
 public:
  virtual ~Catalog() = default;

  // Create the schema if absent; reject a catalog written by a newer schema.
  virtual CatalogStatus initialize() = 0;

  // Insert or replace a session row (keyed by id).
  virtual CatalogStatus upsert_session(const CatalogSession& session) = 0;

  [[nodiscard]] virtual std::vector<CatalogSession> list_sessions() const = 0;
  [[nodiscard]] virtual std::optional<CatalogSession> find_session(const SessionId& id) const = 0;

  virtual CatalogStatus add_tag(const SessionId& id, const std::string& tag) = 0;
  [[nodiscard]] virtual std::vector<std::string> tags_for(const SessionId& id) const = 0;

  virtual CatalogStatus add_annotation(const SessionId& id, const CatalogAnnotation& annotation) = 0;
  [[nodiscard]] virtual std::vector<CatalogAnnotation> annotations_for(const SessionId& id) const = 0;

  virtual CatalogStatus add_bookmark(const SessionId& id, const CatalogBookmark& bookmark) = 0;
  [[nodiscard]] virtual std::vector<CatalogBookmark> bookmarks_for(const SessionId& id) const = 0;

  virtual CatalogStatus add_validation_summary(const SessionId& id,
                                               const ValidationSummary& summary) = 0;
  [[nodiscard]] virtual std::vector<ValidationSummary> validation_summaries_for(
      const SessionId& id) const = 0;

  [[nodiscard]] virtual int schema_version() const = 0;
};

// Fingerprint of a recording file: byte size and a 64-bit FNV-1a hash, rendered as
// hex. Reconstructible and cheap; lets the catalog detect when a file changed.
struct FileFingerprint final {
  bool ok{false};
  std::uint64_t size{0};
  std::string hash;
};

[[nodiscard]] inline FileFingerprint file_fingerprint(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) return {};

  std::uint64_t hash = 1469598103934665603ull;  // FNV-1a offset basis
  std::uint64_t size = 0;
  char buffer[8192];
  while (in) {
    in.read(buffer, sizeof(buffer));
    const std::streamsize got = in.gcount();
    for (std::streamsize i = 0; i < got; ++i) {
      hash ^= static_cast<unsigned char>(buffer[i]);
      hash *= 1099511628211ull;  // FNV prime
    }
    size += static_cast<std::uint64_t>(got);
  }

  static const char* kHex = "0123456789abcdef";
  std::string hex(16, '0');
  for (int i = 15; i >= 0; --i) {
    hex[i] = kHex[hash & 0xF];
    hash >>= 4;
  }
  return {true, size, hex};
}

}  // namespace cavr::catalog
