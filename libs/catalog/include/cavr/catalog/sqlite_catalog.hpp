#pragma once

// SQLite-backed Catalog. The vendored SQLite library is compiled in a single
// translation unit (sqlite_catalog.cpp) and never leaks through this header — the
// sqlite3 handle lives behind a PIMPL, so consumers depend only on cavr::catalog
// and the standard library.
//
// On initialize() the schema (a catalog_meta version table, a sessions table and a
// tags table) is created if absent, and a catalog written by a newer schema
// version is rejected with a clear error.

#include <cavr/catalog/catalog.hpp>

#include <memory>

namespace cavr::catalog {

class SqliteCatalog final : public Catalog {
 public:
  explicit SqliteCatalog(const CatalogOpenOptions& options);
  ~SqliteCatalog() override;

  SqliteCatalog(const SqliteCatalog&) = delete;
  SqliteCatalog& operator=(const SqliteCatalog&) = delete;

  CatalogStatus initialize() override;
  CatalogStatus upsert_session(const CatalogSession& session) override;
  [[nodiscard]] std::vector<CatalogSession> list_sessions() const override;
  [[nodiscard]] std::optional<CatalogSession> find_session(const SessionId& id) const override;
  CatalogStatus add_tag(const SessionId& id, const std::string& tag) override;
  [[nodiscard]] std::vector<std::string> tags_for(const SessionId& id) const override;
  CatalogStatus add_annotation(const SessionId& id, const CatalogAnnotation& annotation) override;
  [[nodiscard]] std::vector<CatalogAnnotation> annotations_for(const SessionId& id) const override;
  CatalogStatus add_bookmark(const SessionId& id, const CatalogBookmark& bookmark) override;
  [[nodiscard]] std::vector<CatalogBookmark> bookmarks_for(const SessionId& id) const override;
  CatalogStatus add_validation_summary(const SessionId& id, const ValidationSummary& summary) override;
  [[nodiscard]] std::vector<ValidationSummary> validation_summaries_for(
      const SessionId& id) const override;
  [[nodiscard]] int schema_version() const override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace cavr::catalog
