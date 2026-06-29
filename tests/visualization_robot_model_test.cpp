#include <cavr/visualization/robot_model.hpp>

#include <array>
#include <cmath>
#include <iostream>
#include <string_view>

namespace {

int failures = 0;

void check(bool condition, std::string_view message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
    ++failures;
  }
}

bool near(double a, double b, double eps = 1.0e-3) {
  return std::abs(a - b) < eps;
}

void test_model_metadata() {
  const auto gp25 = cavr::visualization::yaskawa_gp25();
  check(gp25.joints.size() == 6, "GP25 has six revolute joints");
  check(gp25.asset_path == "assets/robots/yaskawa_gp25/gp25.glb", "asset path points at the glb");
  check(gp25.joints[0].name == "S" && gp25.joints[5].name == "T", "joints are S..T in order");

  using cavr::visualization::deg_to_rad;
  check(near(gp25.joints[0].limit.lower_rad, deg_to_rad(-180.0)), "S lower limit is -180 deg");
  check(near(gp25.joints[5].limit.upper_rad, deg_to_rad(455.0)), "T upper limit is +455 deg");

  // axes: S about +Y, L/U/B about +X, R/T about +Z
  check(near(gp25.joints[0].axis.y_m, 1.0), "S axis is +Y");
  check(near(gp25.joints[1].axis.x_m, 1.0), "L axis is +X");
  check(near(gp25.joints[3].axis.z_m, 1.0), "R axis is +Z");
}

void test_home_pose() {
  const auto gp25 = cavr::visualization::yaskawa_gp25();
  const std::array<double, 6> q{0, 0, 0, 0, 0, 0};
  const auto state = cavr::visualization::forward_kinematics(gp25, q);

  // each joint world origin is the cumulative offset at the home pose
  check(near(state.joints[0].position_m.y_m, 0.169), "S origin at y=0.169");
  check(near(state.joints[2].position_m.y_m, 1.265) &&
        near(state.joints[2].position_m.z_m, 0.150), "U origin at (0,1.265,0.150)");
  check(near(state.joints[4].position_m.z_m, 0.945), "B origin at z=0.945");

  // tool-centre point at the flange face
  check(near(state.tcp.position_m.x_m, 0.0) && near(state.tcp.position_m.y_m, 1.465) &&
        near(state.tcp.position_m.z_m, 1.046), "home TCP at (0,1.465,1.046)");
  check(near(state.tcp.orientation.w(), 1.0), "home TCP orientation is identity");
}

void test_base_rotation() {
  const auto gp25 = cavr::visualization::yaskawa_gp25();
  // S = +90 deg about +Y; everything beyond swings in the XZ plane about the
  // S origin (0,0.169,0). Home TCP (0,1.465,1.046) -> (1.046,1.465,0).
  const std::array<double, 6> q{cavr::visualization::deg_to_rad(90.0), 0, 0, 0, 0, 0};
  const auto state = cavr::visualization::forward_kinematics(gp25, q);
  check(near(state.tcp.position_m.x_m, 1.046), "S=90 swings TCP to x=+1.046");
  check(near(state.tcp.position_m.y_m, 1.465), "S=90 keeps TCP height");
  check(near(state.tcp.position_m.z_m, 0.0), "S=90 swings TCP to z=0");
}

void test_limit_clamp() {
  const auto gp25 = cavr::visualization::yaskawa_gp25();
  const double over = cavr::visualization::deg_to_rad(500.0);
  check(near(gp25.joints[5].limit.clamp(over), gp25.joints[5].limit.upper_rad),
        "T angle beyond range is clamped to the upper limit");
}

}  // namespace

int main() {
  test_model_metadata();
  test_home_pose();
  test_base_rotation();
  test_limit_clamp();

  if (failures != 0) {
    std::cerr << failures << " robot model test(s) failed" << '\n';
    return 1;
  }
  return 0;
}
