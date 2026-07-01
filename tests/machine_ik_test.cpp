// Inverse kinematics on the GP25: for a set of reachable joint configurations,
// FK gives a target pose, and IK (seeded from a different pose) must recover
// joints whose FK matches that pose to tight tolerance. Also checks that IK
// respects joint limits and reports non-convergence for an unreachable target.

#include <cavr/adapters/mock_robot/mock_controller.hpp>
#include <cavr/machine/ik.hpp>
#include <cavr/machine/kinematics.hpp>

#include <cmath>
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

namespace machine = cavr::machine;

double deg(double d) { return d * 3.14159265358979323846 / 180.0; }

// The GP25 axes and the mock's TCP offset (see mock_controller finish_frame).
const std::vector<machine::AxisSpec>& gp25_axes() {
  static const machine::MachineProfile profile = cavr::adapters::mock_robot::make_gp25_profile();
  return profile.axes;
}
constexpr cavr::core::Vec3 kTcpOffset{0.0, 0.0, 0.101};

// Round-trips one configuration: FK(target_q) -> pose, IK(pose, seed) -> q, then
// FK(q) must match the pose (IK may find a different but valid joint solution).
void round_trip(const std::vector<double>& target_q, const std::vector<double>& seed,
                std::string_view label) {
  const auto axes = gp25_axes();
  const cavr::core::Pose3D target = machine::forward_kinematics(axes, target_q, kTcpOffset).tcp;

  const machine::IkResult ik = machine::inverse_kinematics(axes, target, seed, kTcpOffset);
  const std::string ctx = std::string(label) + ": ";
  check(ik.converged, (ctx + "IK converges").c_str());
  check(ik.position_error_m < 1.0e-3, (ctx + "position error < 1 mm").c_str());
  check(ik.orientation_error_rad < 1.0e-2, (ctx + "orientation error < 0.01 rad").c_str());

  // Solution must obey joint limits.
  for (std::size_t i = 0; i < axes.size(); ++i) {
    check(ik.joints[i] >= axes[i].lower_limit - 1e-9 && ik.joints[i] <= axes[i].upper_limit + 1e-9,
          (ctx + "axis " + axes[i].name + " within limits").c_str());
  }

  // FK of the solution reproduces the target pose.
  const cavr::core::Pose3D reached = machine::forward_kinematics(axes, ik.joints, kTcpOffset).tcp;
  const double dx = reached.position_m.x_m - target.position_m.x_m;
  const double dy = reached.position_m.y_m - target.position_m.y_m;
  const double dz = reached.position_m.z_m - target.position_m.z_m;
  check(std::sqrt(dx * dx + dy * dy + dz * dz) < 1.0e-3, (ctx + "FK of IK solution matches target").c_str());
}

void test_reachable_round_trips() {
  round_trip({deg(30), deg(20), deg(-20), 0, deg(40), 0}, {0, 0, 0, 0, 0, 0}, "scan pose");
  round_trip({deg(-40), deg(30), deg(-30), deg(15), deg(50), deg(-20)},
             {deg(10), deg(10), deg(10), 0, deg(10), 0}, "weld pose");
  round_trip({deg(90), deg(-10), deg(10), deg(-30), deg(60), deg(45)}, {0, 0, 0, 0, 0, 0}, "twist pose");
  // Seed already at the answer: must stay converged, near-zero iterations of work.
  round_trip({deg(15), deg(25), deg(-15), 0, deg(35), 0}, {deg(15), deg(25), deg(-15), 0, deg(35), 0}, "seed=answer");
}

void test_unreachable_reports_failure() {
  const auto axes = gp25_axes();
  // A target far outside the GP25's reach (metres away): IK cannot converge.
  cavr::core::Pose3D target;
  target.position_m = {5.0, 5.0, 5.0};
  const machine::IkResult ik = machine::inverse_kinematics(axes, target, {0, 0, 0, 0, 0, 0}, kTcpOffset);
  check(!ik.converged, "unreachable target does not report convergence");
  check(ik.position_error_m > 1.0e-2, "unreachable target keeps a large position error");
  // Even when failing, the returned joints must be within limits.
  for (std::size_t i = 0; i < axes.size(); ++i) {
    check(ik.joints[i] >= axes[i].lower_limit - 1e-9 && ik.joints[i] <= axes[i].upper_limit + 1e-9,
          "failed IK still clamps to limits");
  }
}

}  // namespace

int main() {
  test_reachable_round_trips();
  test_unreachable_reports_failure();

  if (failures != 0) {
    std::cerr << failures << " IK test(s) failed\n";
    return 1;
  }
  std::cout << "machine IK tests passed\n";
  return 0;
}
