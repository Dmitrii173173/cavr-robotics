#pragma once

// Reference ProgramStore backed by a container. Not persistent; pins down the
// semantics the SQLite backend must match and serves as a test double.

#include <cavr/catalog/program_store.hpp>

#include <map>

namespace cavr::catalog {

class InMemoryProgramStore final : public ProgramStore {
 public:
  CatalogStatus initialize() override { return CatalogStatus::success(); }

  CatalogStatus upsert_program(const StoredProgram& program) override {
    if (program.id.empty()) return CatalogStatus::failure("Program id must not be empty");
    programs_[program.id] = program;
    return CatalogStatus::success();
  }

  [[nodiscard]] std::vector<StoredProgram> list_programs() const override {
    std::vector<StoredProgram> out;
    out.reserve(programs_.size());
    for (const auto& [id, program] : programs_) out.push_back(program);
    return out;
  }

  [[nodiscard]] std::optional<StoredProgram> find_program(const std::string& id) const override {
    const auto it = programs_.find(id);
    if (it == programs_.end()) return std::nullopt;
    return it->second;
  }

  CatalogStatus delete_program(const std::string& id) override {
    programs_.erase(id);
    return CatalogStatus::success();
  }

  [[nodiscard]] int schema_version() const override { return kCatalogSchemaVersion; }

 private:
  std::map<std::string, StoredProgram> programs_;  // ordered for stable listing
};

}  // namespace cavr::catalog
