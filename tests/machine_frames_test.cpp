// Coordinate-frame jog math and the tool table: a translation expressed in the
// World, Tool and User frames must move the TCP along the right axes, and the
// tool table must set / clear / select / calibrate as a controller's tool
// registers do.

#include <cavr/machine/frames.hpp>

#include <cmath>
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

namespace machine = cavr::machine;
namespace core = cavr::core;

bool near(double a, double b, double tol = 1e-6) { return std::abs(a - b) < tol; }

double deg(double d) { return d * 3.14159265358979323846 / 180.0; }

// Quaternion for a rotation about +Z by `angle`.
core::Quaternion rot_z(double angle) {
  return core::Quaternion::from_xyzw(0, 0, std::sin(angle / 2), std::cos(angle / 2)).value();
}

void test_world_frame_jog() {
  // TCP at (1,0,0), identity orientation. Jog +0.1 in World X.
  const core::Pose3D tcp{{1, 0, 0}, core::Quaternion::identity()};
  const core::Pose3D out =
      machine::jog_in_frame(tcp, machine::CoordinateSystem::World, {0.1, 0, 0}, {0, 0, 0});
  check(near(out.position_m.x_m, 1.1) && near(out.position_m.y_m, 0) && near(out.position_m.z_m, 0),
        "World jog +X moves along base X");
}

void test_tool_frame_jog() {
  // TCP rotated +90° about Z: its local +X points along base +Y. A +0.1 jog in
  // the Tool frame's X therefore moves the base position along +Y.
  const core::Pose3D tcp{{0, 0, 0}, rot_z(deg(90))};
  const core::Pose3D out =
      machine::jog_in_frame(tcp, machine::CoordinateSystem::Tool, {0.1, 0, 0}, {0, 0, 0});
  check(near(out.position_m.x_m, 0, 1e-6) && near(out.position_m.y_m, 0.1, 1e-6),
        "Tool jog +X moves along the tool's own X (base +Y when tool yawed 90°)");
}

void test_user_frame_jog() {
  // User frame yawed +90° about Z at origin. A +0.1 jog in the User X moves the
  // TCP along the user's X axis, which is base +Y.
  const core::Pose3D user{{0, 0, 0}, rot_z(deg(90))};
  const core::Pose3D tcp{{0, 0, 0}, core::Quaternion::identity()};
  const core::Pose3D out =
      machine::jog_in_frame(tcp, machine::CoordinateSystem::User, {0.1, 0, 0}, {0, 0, 0}, user);
  check(near(out.position_m.x_m, 0, 1e-6) && near(out.position_m.y_m, 0.1, 1e-6),
        "User jog +X moves along the user frame's X (base +Y when user yawed 90°)");
}

void test_compose_invert_round_trip() {
  const core::Pose3D a{{0.2, -0.3, 0.5}, rot_z(deg(30))};
  const core::Pose3D id = machine::compose(machine::invert(a), a);
  check(near(id.position_m.x_m, 0, 1e-6) && near(id.position_m.y_m, 0, 1e-6) &&
            near(id.position_m.z_m, 0, 1e-6),
        "invert(a) ∘ a is the identity translation");
}

void test_tool_table() {
  machine::ToolTable tools;
  check(tools.size() == machine::kToolSlotCount, "tool table has 10 slots");
  check(!tools.slot(3).calibrated, "slots start uncalibrated");

  const core::Pose3D torch{{0, 0, 0.15}, core::Quaternion::identity()};
  tools.set_tool(3, torch, "welding torch");
  check(tools.slot(3).calibrated, "set_tool marks the slot calibrated");
  check(near(tools.slot(3).tcp_offset.position_m.z_m, 0.15), "tool offset stored");
  check(tools.slot(3).name == "welding torch", "tool name stored");

  check(tools.select(3), "select a valid slot");
  check(tools.current() == 3, "current tool updated");
  check(near(tools.current_offset().position_m.z_m, 0.15), "current_offset returns the selected tool");
  check(!tools.select(99), "selecting an out-of-range slot fails");

  tools.clear_tool(3);
  check(!tools.slot(3).calibrated, "clear_tool resets the slot");

  tools.set_tool(0, torch);
  tools.set_tool(9, torch);
  tools.clear_all();
  check(!tools.slot(0).calibrated && !tools.slot(9).calibrated, "clear_all resets every slot");
}

}  // namespace

int main() {
  test_world_frame_jog();
  test_tool_frame_jog();
  test_user_frame_jog();
  test_compose_invert_round_trip();
  test_tool_table();

  if (failures != 0) {
    std::cerr << failures << " frame test(s) failed\n";
    return 1;
  }
  std::cout << "machine frames tests passed\n";
  return 0;
}
