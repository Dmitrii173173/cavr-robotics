#pragma once

// Program registry: persistent, named robot programs (jobs). A job is an ordered
// MotionTask authored on the twin, given a name and saved so it can be reloaded,
// edited and re-run — and later sent to a real controller unchanged. Mirrors the
// robot ProfileStore: an in-memory reference implementation and a SQLite-backed one
// implement the same interface.

#include <cavr/catalog/catalog.hpp>  // CatalogStatus, CatalogOpenOptions
#include <cavr/machine/motion.hpp>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace cavr::catalog {

namespace machine = cavr::machine;

// One saved job: the ordered motion task plus which robot it was authored for.
struct StoredProgram final {
  std::string id;                 // registry key (stable, e.g. "weld_seam_a")
  std::string name;               // human label shown in the UI
  std::string robot_id;           // the robot (profile) this job targets ("" = any)
  machine::MotionTask task;       // the ordered program steps
  std::int64_t updated_ns{0};     // last write, ns since epoch
};

// Engine-neutral program registry.
class ProgramStore {
 public:
  virtual ~ProgramStore() = default;

  virtual CatalogStatus initialize() = 0;
  virtual CatalogStatus upsert_program(const StoredProgram& program) = 0;

  [[nodiscard]] virtual std::vector<StoredProgram> list_programs() const = 0;
  [[nodiscard]] virtual std::optional<StoredProgram> find_program(const std::string& id) const = 0;

  virtual CatalogStatus delete_program(const std::string& id) = 0;

  [[nodiscard]] virtual int schema_version() const = 0;
};

}  // namespace cavr::catalog
