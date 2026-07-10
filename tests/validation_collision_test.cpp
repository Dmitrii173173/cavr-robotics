// Collision checking in the validator: the planned trajectory is sampled and each
// configuration is checked against the robot itself, the floor, and sphere
// obstacles. We assert both directions — a benign task with a sane cell passes
// with collisions_evaluated set, and each hazard (floor, obstacle, self) is caught.

#include <cavr/validation/collision.hpp>
#include <cavr/validation/trajectory_validator.hpp>

#include <cavr/adapters/mock_robot/mock_controller.hpp>
#include <cavr/machine/kinematics.hpp>
#include <cavr/machine/motion.hpp>

#include <iostream>
#include <string>
#include <string_view>
#include <vector>

namespace {

int failures = 0;

void check(bool condition, std::string_view message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
    ++failures;
  }
}

namespace validation = cavr::validation;
namespace machine = cavr::machine;
using cavr::core::Pose3D;
using cavr::core::Vec3;

bool has_collision_issue(const validation::ValidationReport& r, std::string_view needle) {
  for (const auto& i : r.issues) {
    if (i.message.rfind("Collision:", 0) == 0 && i.message.find(needle) != std::string::npos) return true;
  }
  return false;
}

bool any_collision_issue(const validation::ValidationReport& r) {
  for (const auto& i : r.issues) {
    if (i.message.rfind("Collision:", 0) == 0) return true;
  }
  return false;
}

// A short, in-limits joint move — a valid trajectory to sample.
machine::MotionTask benign_task() {
  machine::MotionCommand c;
  c.kind = machine::MotionKind::MoveJ;
  c.target.joints = std::vector<double>{0.3, -0.2, 0.1, 0.0, 0.0, 0.0};
  c.speed = 0.5;
  return {c};
}

Pose3D tcp_tool(const machine::MachineProfile& p) {
  if (const machine::CoordinateFrame* tcp = p.frame("tcp")) return tcp->transform;
  return {};
}

// The base overload does not evaluate collisions and says so.
void test_base_overload_honest() {
  const auto profile = cavr::adapters::mock_robot::make_gp25_profile();
  const auto report = validation::validate_task(profile, benign_task());
  check(!report.collisions_evaluated, "base validate_task leaves collisions_evaluated false");
}

// A sane cell (floor well below the Y-up robot, no obstacles) passes with no
// collision issues, and now reports collisions as evaluated.
void test_clean_pass() {
  const auto profile = cavr::adapters::mock_robot::make_gp25_profile();
  validation::CollisionModel model;
  model.check_self = false;      // exercised on its own below
  model.check_floor = true;
  model.floor_up_axis = 1;       // GP25 is Y-up
  model.floor_level_m = -2.0;    // floor far beneath the robot
  const auto report = validation::validate_task(profile, benign_task(), model);
  check(report.collisions_evaluated, "collisions_evaluated is set by the model overload");
  check(!any_collision_issue(report), "a clean cell yields no collision issues");
}

// A floor plane raised above the robot forces every link below it.
void test_floor_hit() {
  const auto profile = cavr::adapters::mock_robot::make_gp25_profile();
  validation::CollisionModel model;
  model.check_self = false;
  model.check_floor = true;
  model.floor_up_axis = 1;
  model.floor_level_m = 2.0;     // above the whole robot
  const auto report = validation::validate_task(profile, benign_task(), model);
  check(has_collision_issue(report, "floor"), "a raised floor is detected as a collision");
  check(!report.ok(), "a collision makes the report not ok");
}

// A sphere sitting on the tool at the home pose is hit.
void test_obstacle_hit() {
  const auto profile = cavr::adapters::mock_robot::make_gp25_profile();
  const auto fk = machine::forward_kinematics(profile.axes, std::vector<double>(profile.dof(), 0.0),
                                              tcp_tool(profile));
  validation::CollisionModel model;
  model.check_self = false;
  model.check_floor = false;
  model.spheres.push_back({fk.tcp.position_m, 0.25, "fixture"});
  const auto report = validation::validate_task(profile, benign_task(), model);
  check(has_collision_issue(report, "fixture"), "an obstacle sphere on the tool path is detected");
}

// An exaggerated link radius makes non-adjacent links overlap: self-collision.
void test_self_collision() {
  const auto profile = cavr::adapters::mock_robot::make_gp25_profile();
  validation::CollisionModel model;
  model.check_self = true;
  model.check_floor = false;
  model.link_radius_m = 0.5;     // huge capsules → non-adjacent links interfere
  const auto report = validation::validate_task(profile, benign_task(), model);
  check(any_collision_issue(report), "oversized links trigger self-collision");
  // A self-collision names two links (neither is 'floor' or an obstacle).
  bool link_pair = false;
  for (const auto& i : report.issues) {
    if (i.message.rfind("Collision:", 0) == 0 && i.message.find("floor") == std::string::npos &&
        i.message.find("fixture") == std::string::npos)
      link_pair = true;
  }
  check(link_pair, "self-collision reports a link-vs-link pair");
}

}  // namespace

int main() {
  test_base_overload_honest();
  test_clean_pass();
  test_floor_hit();
  test_obstacle_hit();
  test_self_collision();

  if (failures != 0) {
    std::cerr << failures << " collision validation test(s) failed\n";
    return 1;
  }
  std::cout << "collision validation tests passed\n";
  return 0;
}
