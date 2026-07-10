// Hand-eye solver (Tsai-Lenz): a closed-loop synthetic test. We pick a known true
// transform X, generate a set of robot poses, synthesize the exact camera
// observations the geometry implies, and require the solver to recover X (and
// report a near-zero residual). Because the data is noise-free and AX = XB holds
// exactly, recovery should be accurate to near machine precision.

#include <cavr/calibration/hand_eye_solver.hpp>

#include <cavr/machine/frames.hpp>

#include <cmath>
#include <iostream>
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

namespace cal = cavr::calibration;
namespace machine = cavr::machine;
using cavr::core::Pose3D;
using cavr::core::Quaternion;
using cavr::core::Vec3;

// Quaternion from an axis (need not be unit) and angle in radians.
Quaternion quat(double ax, double ay, double az, double angle) {
  const double n = std::sqrt(ax * ax + ay * ay + az * az);
  if (n < 1e-12) return Quaternion::identity();
  const double h = angle * 0.5, s = std::sin(h) / n;
  return Quaternion::from_xyzw(ax * s, ay * s, az * s, std::cos(h), 1e-6).value();
}

double pos_dist(const Vec3& a, const Vec3& b) {
  return std::sqrt((a.x_m - b.x_m) * (a.x_m - b.x_m) + (a.y_m - b.y_m) * (a.y_m - b.y_m) +
                   (a.z_m - b.z_m) * (a.z_m - b.z_m));
}

// Angle (rad) between two orientations.
double angle_between(const Quaternion& a, const Quaternion& b) {
  const Pose3D pa{{}, a}, pb{{}, b};
  const Quaternion d = machine::compose(machine::invert(pa), pb).orientation;
  return 2.0 * std::acos(std::min(1.0, std::abs(d.w())));
}

// A spread of robot flange poses with non-parallel rotation axes (Tsai-Lenz needs
// rotation diversity to be well-conditioned) and varied translations.
std::vector<Pose3D> flange_poses() {
  return {
      Pose3D{Vec3{0.60, 0.00, 0.90}, quat(1, 0, 0, 0.5)},
      Pose3D{Vec3{0.55, 0.10, 0.85}, quat(0, 1, 0, -0.6)},
      Pose3D{Vec3{0.65, -0.10, 0.95}, quat(0, 0, 1, 0.7)},
      Pose3D{Vec3{0.50, 0.15, 0.80}, quat(1, 1, 0, 0.4)},
      Pose3D{Vec3{0.70, -0.05, 1.00}, quat(0, 1, 1, -0.5)},
      Pose3D{Vec3{0.58, 0.08, 0.88}, quat(1, 0, 1, 0.55)},
  };
}

// Eye-in-hand: camera rigidly mounted on the flange, viewing a target fixed in the
// base. The target pose cancels out of AX=XB, so we may take it as the origin:
// target_in_camera = (flange · X)^-1.
void test_eye_in_hand() {
  const Pose3D true_x{Vec3{0.05, -0.02, 0.08}, quat(0.3, 0.6, 0.2, 0.35)};  // camera-in-flange

  std::vector<cal::HandEyeSample> samples;
  for (const Pose3D& flange : flange_poses()) {
    const Pose3D camera_in_base = machine::compose(flange, true_x);
    samples.push_back({flange, machine::invert(camera_in_base)});
  }

  const auto result = cal::solve_hand_eye(samples, cal::HandEyeType::EyeInHand, "weld_cam");
  check(result.ok, "eye-in-hand solve succeeds");
  check(result.pairs_used == 5, "uses one motion per consecutive pair");
  check(result.calibration.type == cal::HandEyeType::EyeInHand, "result is tagged eye-in-hand");
  check(result.calibration.camera_frame == "weld_cam", "camera frame is carried through");

  check(pos_dist(result.calibration.transform.position_m, true_x.position_m) < 1e-6,
        "recovers the true camera translation");
  check(angle_between(result.calibration.transform.orientation, true_x.orientation) < 1e-5,
        "recovers the true camera rotation");
  check(result.calibration.residual_position_m < 1e-6 && result.calibration.residual_rotation_rad < 1e-6,
        "reports a near-zero AX=XB residual");
  check(result.calibration.method == "tsai-lenz", "records the method");
}

// Eye-to-hand: camera fixed in the cell (X = camera-in-base). With the solver's
// internal inversion of the flange poses, target_in_camera = X^-1 · flange.
void test_eye_to_hand() {
  const Pose3D true_x{Vec3{1.00, 0.20, 1.50}, quat(0.1, 0.2, 0.9, -0.4)};  // camera-in-base

  std::vector<cal::HandEyeSample> samples;
  for (const Pose3D& flange : flange_poses()) {
    samples.push_back({flange, machine::compose(machine::invert(true_x), flange)});
  }

  const auto result = cal::solve_hand_eye(samples, cal::HandEyeType::EyeToHand, "cell_cam");
  check(result.ok, "eye-to-hand solve succeeds");
  check(pos_dist(result.calibration.transform.position_m, true_x.position_m) < 1e-6,
        "recovers the true fixed-camera translation");
  check(angle_between(result.calibration.transform.orientation, true_x.orientation) < 1e-5,
        "recovers the true fixed-camera rotation");
}

// Too few samples is rejected, not silently wrong.
void test_insufficient_samples() {
  std::vector<cal::HandEyeSample> two(2);
  const auto result = cal::solve_hand_eye(two);
  check(!result.ok, "fewer than 3 samples is rejected");
}

}  // namespace

int main() {
  test_eye_in_hand();
  test_eye_to_hand();
  test_insufficient_samples();

  if (failures != 0) {
    std::cerr << failures << " hand-eye solver test(s) failed\n";
    return 1;
  }
  std::cout << "hand-eye solver tests passed\n";
  return 0;
}
