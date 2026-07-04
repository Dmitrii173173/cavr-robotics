#pragma once

// SQLite-backed ProgramStore. Compiled in the same catalog target as the other
// SQLite backends; the sqlite3 handle lives behind a PIMPL so consumers depend
// only on cavr::catalog, cavr::machine and the standard library.

#include <cavr/catalog/program_store.hpp>

#include <memory>

namespace cavr::catalog {

class SqliteProgramStore final : public ProgramStore {
 public:
  explicit SqliteProgramStore(const CatalogOpenOptions& options);
  ~SqliteProgramStore() override;

  SqliteProgramStore(const SqliteProgramStore&) = delete;
  SqliteProgramStore& operator=(const SqliteProgramStore&) = delete;

  CatalogStatus initialize() override;
  CatalogStatus upsert_program(const StoredProgram& program) override;
  [[nodiscard]] std::vector<StoredProgram> list_programs() const override;
  [[nodiscard]] std::optional<StoredProgram> find_program(const std::string& id) const override;
  CatalogStatus delete_program(const std::string& id) override;
  [[nodiscard]] int schema_version() const override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace cavr::catalog
