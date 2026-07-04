#pragma once

// Robot registry: persistent, named MachineProfiles so any robot can be added,
// named and reused without re-discovery. Unlike the session Catalog (a cache of
// reconstructible metadata), the registry IS a source of truth — it holds the
// authored/discovered profile plus how to reach the controller (adapter +
// endpoint), so the UI can list robots, pick one and connect it.
//
// The storage engine is hidden behind the ProfileStore interface: an in-memory
// reference implementation and a SQLite-backed one implement it identically,
// exactly as Catalog / SqliteCatalog do.

#include <cavr/catalog/catalog.hpp>  // CatalogStatus, CatalogOpenOptions
#include <cavr/machine/machine_profile.hpp>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace cavr::catalog {

namespace machine = cavr::machine;

// One registered robot: the controller-neutral profile plus the connection
// descriptor that tells the adapter factory how to reach it.
struct StoredRobot final {
  std::string id;                 // registry key (stable, e.g. "pnr_cell_a")
  std::string display_name;       // human label shown in the UI
  machine::MachineProfile profile;

  std::string adapter{"mock"};    // "mock" | "generic_tcp" | ... (adapter factory key)
  std::string transport{"mock"};  // "mock" | "tcp" | ...
  std::string endpoint;           // host:port / device path ("" for mock)

  std::int64_t updated_ns{0};     // last write, ns since epoch
};

// Engine-neutral robot registry. Implementations persist (or not) however they like.
class ProfileStore {
 public:
  virtual ~ProfileStore() = default;

  // Create the schema if absent; reject a store written by a newer schema.
  virtual CatalogStatus initialize() = 0;

  // Insert or replace a robot row (keyed by id).
  virtual CatalogStatus upsert_robot(const StoredRobot& robot) = 0;

  [[nodiscard]] virtual std::vector<StoredRobot> list_robots() const = 0;
  [[nodiscard]] virtual std::optional<StoredRobot> find_robot(const std::string& id) const = 0;

  virtual CatalogStatus delete_robot(const std::string& id) = 0;

  [[nodiscard]] virtual int schema_version() const = 0;
};

}  // namespace cavr::catalog
