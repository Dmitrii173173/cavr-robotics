#pragma once

// Reference ProfileStore backed by an ordinary container. Not persistent — it
// pins down the semantics the SQLite backend must match, and serves as a test
// double, mirroring InMemoryCatalog.

#include <cavr/catalog/profile_store.hpp>

#include <map>

namespace cavr::catalog {

class InMemoryProfileStore final : public ProfileStore {
 public:
  CatalogStatus initialize() override { return CatalogStatus::success(); }

  CatalogStatus upsert_robot(const StoredRobot& robot) override {
    if (robot.id.empty()) return CatalogStatus::failure("Robot id must not be empty");
    robots_[robot.id] = robot;
    return CatalogStatus::success();
  }

  [[nodiscard]] std::vector<StoredRobot> list_robots() const override {
    std::vector<StoredRobot> out;
    out.reserve(robots_.size());
    for (const auto& [id, robot] : robots_) out.push_back(robot);
    return out;
  }

  [[nodiscard]] std::optional<StoredRobot> find_robot(const std::string& id) const override {
    const auto it = robots_.find(id);
    if (it == robots_.end()) return std::nullopt;
    return it->second;
  }

  CatalogStatus delete_robot(const std::string& id) override {
    robots_.erase(id);
    return CatalogStatus::success();
  }

  [[nodiscard]] int schema_version() const override { return kCatalogSchemaVersion; }

 private:
  std::map<std::string, StoredRobot> robots_;  // ordered for stable listing
};

}  // namespace cavr::catalog
