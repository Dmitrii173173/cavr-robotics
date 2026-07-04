#pragma once

// SQLite-backed ProfileStore. The vendored SQLite library is compiled in the same
// single translation unit as SqliteCatalog (sqlite_profile_store.cpp includes
// sqlite3.h); the handle lives behind a PIMPL so consumers depend only on
// cavr::catalog, cavr::machine and the standard library.
//
// On initialize() the `robots` table is created if absent (sharing the
// catalog_meta schema-version row), and a store written by a newer schema version
// is rejected with a clear error.

#include <cavr/catalog/profile_store.hpp>

#include <memory>

namespace cavr::catalog {

class SqliteProfileStore final : public ProfileStore {
 public:
  explicit SqliteProfileStore(const CatalogOpenOptions& options);
  ~SqliteProfileStore() override;

  SqliteProfileStore(const SqliteProfileStore&) = delete;
  SqliteProfileStore& operator=(const SqliteProfileStore&) = delete;

  CatalogStatus initialize() override;
  CatalogStatus upsert_robot(const StoredRobot& robot) override;
  [[nodiscard]] std::vector<StoredRobot> list_robots() const override;
  [[nodiscard]] std::optional<StoredRobot> find_robot(const std::string& id) const override;
  CatalogStatus delete_robot(const std::string& id) override;
  [[nodiscard]] int schema_version() const override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace cavr::catalog
