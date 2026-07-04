// ProgramStore (job registry) contract: the same semantics must hold for every
// engine, and a job with mixed step kinds (MoveJ/MoveL/MoveC/Wait/ToolOn) must
// survive the JSON round-trip the store relies on, including its poses and via.

#include <cavr/catalog/in_memory_program_store.hpp>

#include <cavr/machine/motion.hpp>

#ifdef CAVR_WITH_SQLITE
#include <cavr/catalog/sqlite_program_store.hpp>
#endif

#include <cmath>
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
namespace core = cavr::core;

// A representative job: a joint move, a straight move, an arc with a via, a wait,
// and a tool-on — one of each shape the serializer must preserve.
machine::MotionTask sample_task() {
  machine::MotionTask t;

  machine::MotionCommand mj;
  mj.kind = machine::MotionKind::MoveJ;
  mj.target.joints = std::vector<double>{0.1, -0.2, 0.3, 0.0, 0.4, 0.0};
  mj.speed = 1.0;
  mj.label = "home";
  t.push_back(mj);

  machine::MotionCommand ml;
  ml.kind = machine::MotionKind::MoveL;
  ml.target.pose = core::Pose3D{core::Vec3{0.4, 0.1, 0.6}, core::Quaternion::identity()};
  ml.speed = 250.0;
  ml.label = "approach";
  t.push_back(ml);

  machine::MotionCommand mc;
  mc.kind = machine::MotionKind::MoveC;
  mc.via = core::Pose3D{core::Vec3{0.45, 0.15, 0.62}, core::Quaternion::identity()};
  mc.target.pose = core::Pose3D{core::Vec3{0.5, 0.0, 0.6}, core::Quaternion::identity()};
  mc.speed = 200.0;
  mc.label = "arc";
  t.push_back(mc);

  machine::MotionCommand wait;
  wait.kind = machine::MotionKind::Wait;
  wait.wait_s = 1.5;
  t.push_back(wait);

  machine::MotionCommand on;
  on.kind = machine::MotionKind::ToolOn;
  t.push_back(on);

  return t;
}

catalog::StoredProgram make_stored(std::string id, std::string name) {
  catalog::StoredProgram p;
  p.id = std::move(id);
  p.name = std::move(name);
  p.robot_id = "gp25_cell1";
  p.task = sample_task();
  p.updated_ns = 7;
  return p;
}

double dist(const core::Vec3& a, const core::Vec3& b) {
  const double dx = a.x_m - b.x_m, dy = a.y_m - b.y_m, dz = a.z_m - b.z_m;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void run_contract(catalog::ProgramStore& store, std::string_view engine) {
  const std::string ctx = std::string(engine) + ": ";
  check(static_cast<bool>(store.initialize()), (ctx + "initialize succeeds").c_str());

  check(static_cast<bool>(store.upsert_program(make_stored("weld_a", "Weld seam A"))),
        (ctx + "insert program").c_str());
  check(static_cast<bool>(store.upsert_program(make_stored("weld_b", "Weld seam B"))),
        (ctx + "insert second program").c_str());

  const auto all = store.list_programs();
  check(all.size() == 2 && all[0].id == "weld_a" && all[1].id == "weld_b",
        (ctx + "lists programs ordered by id").c_str());

  const auto got = store.find_program("weld_a");
  check(got.has_value(), (ctx + "program reads back").c_str());
  check(got && got->name == "Weld seam A" && got->robot_id == "gp25_cell1",
        (ctx + "metadata round-trips").c_str());

  // The task round-trips: step count, kinds, a pose and a via.
  check(got && got->task.size() == 5, (ctx + "step count round-trips").c_str());
  if (got && got->task.size() == 5) {
    check(got->task[0].kind == machine::MotionKind::MoveJ && got->task[0].target.joints &&
              got->task[0].target.joints->size() == 6,
          (ctx + "MoveJ joints round-trip").c_str());
    check(got->task[1].kind == machine::MotionKind::MoveL && got->task[1].target.pose &&
              dist(got->task[1].target.pose->position_m, core::Vec3{0.4, 0.1, 0.6}) < 1e-9,
          (ctx + "MoveL pose round-trips").c_str());
    check(got->task[2].kind == machine::MotionKind::MoveC && got->task[2].via &&
              dist(got->task[2].via->position_m, core::Vec3{0.45, 0.15, 0.62}) < 1e-9,
          (ctx + "MoveC via round-trips").c_str());
    check(std::abs(got->task[3].wait_s - 1.5) < 1e-9, (ctx + "Wait seconds round-trip").c_str());
    check(got->task[4].kind == machine::MotionKind::ToolOn, (ctx + "ToolOn round-trips").c_str());
  }

  // upsert replaces, delete removes.
  auto renamed = make_stored("weld_a", "Weld A v2");
  check(static_cast<bool>(store.upsert_program(renamed)), (ctx + "upsert replaces").c_str());
  check(store.find_program("weld_a")->name == "Weld A v2", (ctx + "replace updates name").c_str());
  check(store.list_programs().size() == 2, (ctx + "upsert did not duplicate").c_str());

  check(static_cast<bool>(store.delete_program("weld_b")), (ctx + "delete program").c_str());
  check(!store.find_program("weld_b").has_value(), (ctx + "deleted program is gone").c_str());
  check(store.list_programs().size() == 1, (ctx + "one program remains").c_str());

  check(!static_cast<bool>(store.upsert_program(make_stored("", "no id"))),
        (ctx + "empty id rejected").c_str());
}

#ifdef CAVR_WITH_SQLITE
std::filesystem::path temp_db(const char* name) {
  return std::filesystem::temp_directory_path() / name;
}

void test_sqlite_persistence() {
  const auto path = temp_db("cavr_programs_persist.db");
  std::filesystem::remove(path);
  {
    catalog::SqliteProgramStore store({path.string(), true});
    check(static_cast<bool>(store.initialize()), "sqlite: init for persistence");
    check(static_cast<bool>(store.upsert_program(make_stored("weld_a", "Persisted"))),
          "sqlite: insert for persistence");
  }
  {
    catalog::SqliteProgramStore store({path.string(), true});
    check(static_cast<bool>(store.initialize()), "sqlite: reopen existing registry");
    const auto got = store.find_program("weld_a");
    check(got && got->name == "Persisted", "sqlite: program survived reopen");
    check(got && got->task.size() == 5, "sqlite: task survived reopen");
  }
  std::filesystem::remove(path);
}
#endif

}  // namespace

int main() {
  {
    catalog::InMemoryProgramStore store;
    run_contract(store, "in-memory");
  }
#ifdef CAVR_WITH_SQLITE
  {
    const auto path = temp_db("cavr_programs_contract.db");
    std::filesystem::remove(path);
    catalog::SqliteProgramStore store({path.string(), true});
    run_contract(store, "sqlite");
    std::filesystem::remove(path);
  }
  test_sqlite_persistence();
#endif

  if (failures == 0) {
    std::cout << "All program store tests passed\n";
    return 0;
  }
  std::cerr << failures << " check(s) failed\n";
  return 1;
}
