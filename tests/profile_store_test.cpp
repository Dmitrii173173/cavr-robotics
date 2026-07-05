// ProfileStore (robot registry) contract: the same semantics must hold for every
// engine. The in-memory reference and the SQLite backend are driven through the
// ProfileStore interface by one shared body, plus SQLite persistence across reopen.
// A separate check confirms the PNR IO banks (IoKind::Group / IoDirection::Internal)
// survive the JSON round-trip the store relies on.

#include <cavr/catalog/in_memory_profile_store.hpp>

#include <cavr/adapters/mock_robot/mock_controller.hpp>
#include <cavr/machine/profile_io.hpp>

#ifdef CAVR_WITH_SQLITE
#include <cavr/catalog/sqlite_profile_store.hpp>
#endif

#include <filesystem>
#include <iostream>
#include <string>
#include <string_view>

namespace {

int failures = 0;

void check(bool condition, std::string_view message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
    ++failures;
  }
}

namespace catalog = cavr::catalog;
namespace machine = cavr::machine;
namespace mock = cavr::adapters::mock_robot;

catalog::StoredRobot make_stored(std::string id, std::string name, machine::MachineProfile profile,
                                 std::string adapter, std::string endpoint) {
  catalog::StoredRobot r;
  r.id = std::move(id);
  r.display_name = std::move(name);
  r.profile = std::move(profile);
  r.adapter = std::move(adapter);
  r.transport = r.adapter == "mock" ? "mock" : "tcp";
  r.endpoint = std::move(endpoint);
  r.updated_ns = 42;
  return r;
}

// Count IO channels of a given kind — used to confirm PNR's Group bank round-trips.
int count_kind(const machine::MachineProfile& p, machine::IoKind kind) {
  int n = 0;
  for (const auto& c : p.io)
    if (c.kind == kind) ++n;
  return n;
}

// The engine-independent contract, run against any ProfileStore.
void run_contract(catalog::ProfileStore& store, std::string_view engine) {
  const std::string ctx = std::string(engine) + ": ";

  check(static_cast<bool>(store.initialize()), (ctx + "initialize succeeds").c_str());

  const auto gp25 = mock::make_gp25_profile();
  const auto pnr = mock::make_pnr_profile();

  check(static_cast<bool>(store.upsert_robot(
            make_stored("gp25", "GP25 cell", gp25, "mock", ""))),
        (ctx + "insert gp25").c_str());
  check(static_cast<bool>(store.upsert_robot(
            make_stored("pnr", "PNR cell", pnr, "generic_tcp", "10.0.0.5:5000"))),
        (ctx + "insert pnr").c_str());

  // list is ordered by id: gp25 then pnr
  const auto all = store.list_robots();
  check(all.size() == 2, (ctx + "lists both robots").c_str());
  check(all.size() == 2 && all[0].id == "gp25" && all[1].id == "pnr",
        (ctx + "list ordered by id").c_str());

  // find + connection descriptor round-trip
  const auto got = store.find_robot("pnr");
  check(got.has_value(), (ctx + "pnr reads back").c_str());
  check(got && got->display_name == "PNR cell" && got->adapter == "generic_tcp" &&
            got->endpoint == "10.0.0.5:5000" && got->transport == "tcp",
        (ctx + "connection descriptor round-trips").c_str());

  // profile round-trip: dof + IO banks including the Group kind and Internal dir
  check(got && got->profile.dof() == pnr.dof(), (ctx + "pnr dof round-trips").c_str());
  check(got && got->profile.io.size() == pnr.io.size(), (ctx + "pnr io count round-trips").c_str());
  check(got && count_kind(got->profile, machine::IoKind::Group) == 2,
        (ctx + "pnr Group bank round-trips").c_str());
  bool has_internal = false;
  if (got)
    for (const auto& c : got->profile.io)
      if (c.direction == machine::IoDirection::Internal) has_internal = true;
  check(has_internal, (ctx + "pnr Internal (M) direction round-trips").c_str());
  check(got && got->profile.robot_model == "PNR PR6-900", (ctx + "robot_model round-trips").c_str());

  // upsert replaces
  check(static_cast<bool>(store.upsert_robot(make_stored("pnr", "PNR renamed", pnr, "mock", ""))),
        (ctx + "upsert replaces").c_str());
  const auto renamed = store.find_robot("pnr");
  check(renamed && renamed->display_name == "PNR renamed" && renamed->adapter == "mock",
        (ctx + "replaced fields update").c_str());
  check(store.list_robots().size() == 2, (ctx + "upsert did not duplicate").c_str());

  // delete
  check(static_cast<bool>(store.delete_robot("gp25")), (ctx + "delete gp25").c_str());
  check(!store.find_robot("gp25").has_value(), (ctx + "gp25 is gone").c_str());
  check(store.list_robots().size() == 1, (ctx + "one robot remains").c_str());

  // empty id is rejected
  check(!static_cast<bool>(store.upsert_robot(make_stored("", "no id", gp25, "mock", ""))),
        (ctx + "empty id rejected").c_str());
}

// Direct JSON round-trip of the new PNR IO enums, independent of the store.
void test_enum_round_trip() {
  const auto pnr = mock::make_pnr_profile();
  const auto reparsed = machine::parse_profile(machine::export_profile_string(pnr)).profile;
  check(reparsed.io.size() == pnr.io.size(), "enum: io count round-trips");
  check(count_kind(reparsed, machine::IoKind::Group) == count_kind(pnr, machine::IoKind::Group),
        "enum: Group kind survives export/parse");
  bool internal = false;
  for (const auto& c : reparsed.io)
    if (c.direction == machine::IoDirection::Internal) internal = true;
  check(internal, "enum: Internal direction survives export/parse");
}

#ifdef CAVR_WITH_SQLITE
std::filesystem::path temp_db(const char* name) {
  return std::filesystem::temp_directory_path() / name;
}

// Robots must survive closing and reopening the database file.
void test_sqlite_persistence() {
  const auto path = temp_db("cavr_registry_persist.db");
  std::filesystem::remove(path);
  {
    catalog::SqliteProfileStore store({path.string(), true});
    check(static_cast<bool>(store.initialize()), "sqlite: init for persistence");
    check(static_cast<bool>(store.upsert_robot(
              make_stored("pnr", "PNR persisted", mock::make_pnr_profile(), "mock", ""))),
          "sqlite: insert for persistence");
  }
  {
    catalog::SqliteProfileStore store({path.string(), true});
    check(static_cast<bool>(store.initialize()), "sqlite: reopen existing registry");
    const auto got = store.find_robot("pnr");
    check(got && got->display_name == "PNR persisted", "sqlite: robot survived reopen");
    check(got && count_kind(got->profile, machine::IoKind::Group) == 2,
          "sqlite: profile IO survived reopen");
  }
  std::filesystem::remove(path);
}
#endif

}  // namespace

int main() {
  {
    catalog::InMemoryProfileStore store;
    run_contract(store, "in-memory");
  }
  test_enum_round_trip();

#ifdef CAVR_WITH_SQLITE
  {
    const auto path = temp_db("cavr_registry_contract.db");
    std::filesystem::remove(path);
    {
      catalog::SqliteProfileStore store({path.string(), true});
      run_contract(store, "sqlite");
    }
    std::filesystem::remove(path);
  }
  test_sqlite_persistence();
#endif

  if (failures == 0) {
    std::cout << "All profile store tests passed\n";
    return 0;
  }
  std::cerr << failures << " check(s) failed\n";
  return 1;
}
