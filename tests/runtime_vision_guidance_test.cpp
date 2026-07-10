// Vision-guided correction, end to end: a synthetic scan cloud in the camera frame
// is placed into the base frame with a known hand-eye transform + flange pose, its
// seam (centroid) is compared to where the plan expected it, and the resulting
// offset is applied to a MotionTask. We assert the whole chain: the cloud lands
// where the geometry says it should, the offset is exact, and only Cartesian
// targets move.

#include <cavr/runtime/vision_guidance.hpp>

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

namespace runtime = cavr::runtime;
namespace machine = cavr::machine;
namespace cal = cavr::calibration;
namespace sdk = cavr::adapter_sdk;
using cavr::core::Pose3D;
using cavr::core::Quaternion;
using cavr::core::Vec3;

double vdist(const Vec3& a, const Vec3& b) {
  return std::sqrt((a.x_m - b.x_m) * (a.x_m - b.x_m) + (a.y_m - b.y_m) * (a.y_m - b.y_m) +
                   (a.z_m - b.z_m) * (a.z_m - b.z_m));
}

Quaternion quat(double ax, double ay, double az, double angle) {
  const double n = std::sqrt(ax * ax + ay * ay + az * az);
  if (n < 1e-12) return Quaternion::identity();
  const double h = angle * 0.5, s = std::sin(h) / n;
  return Quaternion::from_xyzw(ax * s, ay * s, az * s, std::cos(h), 1e-6).value();
}

// A single scan point in the camera frame maps to the base frame exactly as
// camera_in_base composed with it.
void test_cloud_to_base() {
  cal::HandEyeCalibration he;
  he.type = cal::HandEyeType::EyeInHand;
  he.camera_frame = "weld_cam";
  he.transform = Pose3D{Vec3{0.05, -0.02, 0.08}, quat(0.2, 0.4, 0.1, 0.3)};

  const Pose3D flange{Vec3{0.6, 0.1, 0.9}, quat(0, 0, 1, 0.25)};

  sdk::PointCloud cam;
  cam.frame_id = "weld_cam";
  cam.points = {Vec3{0.0, 0.0, 0.5}, Vec3{0.02, -0.01, 0.55}};

  const sdk::PointCloud base = runtime::cloud_in_base(cam, he, flange);
  check(base.frame_id == "base", "transformed cloud is stamped base");
  check(base.size() == cam.size(), "point count is preserved");

  const Pose3D cam_in_base = cal::camera_in_base(he, flange);
  for (std::size_t i = 0; i < cam.points.size(); ++i) {
    const Vec3 expected =
        machine::compose(cam_in_base, Pose3D{cam.points[i], Quaternion::identity()}).position_m;
    check(vdist(base.points[i], expected) < 1e-12, "each point maps through camera_in_base");
  }
}

// The seam offset is exactly the displacement between the observed centroid and the
// plan's expected seam point.
void test_seam_offset_and_correction() {
  cal::HandEyeCalibration he;
  he.type = cal::HandEyeType::EyeInHand;
  he.camera_frame = "weld_cam";
  he.transform = Pose3D{Vec3{0.0, 0.0, 0.10}, Quaternion::identity()};  // camera 10cm past the flange

  // Flange oriented so the camera looks straight down +Z of base, at a known spot.
  const Pose3D flange{Vec3{0.50, 0.00, 1.00}, Quaternion::identity()};

  // A tight scan cluster around one point in the camera frame. Its base-frame
  // centroid is where the seam actually is.
  sdk::PointCloud cam;
  cam.frame_id = "weld_cam";
  cam.points = {Vec3{0.01, 0.00, 0.30}, Vec3{-0.01, 0.00, 0.30}, Vec3{0.00, 0.02, 0.30},
                Vec3{0.00, -0.02, 0.30}};

  const sdk::PointCloud base = runtime::cloud_in_base(cam, he, flange);
  const Vec3 observed = runtime::centroid(base);
  // camera_in_base = flange ∘ (0,0,0.10); centroid of cam points = (0,0,0.30) →
  // base (0.50, 0.00, 1.00+0.10+0.30) = (0.50, 0.00, 1.40).
  check(vdist(observed, Vec3{0.50, 0.00, 1.40}) < 1e-9, "observed seam centroid is where geometry predicts");

  // The plan thought the seam was 8 mm off in X and 5 mm in Y.
  const Vec3 expected_seam{0.492, -0.005, 1.40};
  const Vec3 offset = runtime::seam_offset(base, expected_seam);
  check(vdist(offset, Vec3{0.008, 0.005, 0.0}) < 1e-9, "seam offset is observed - expected");

  // A weld task: MoveJ approach (joint space), then MoveL along the nominal seam.
  machine::MotionTask task;
  machine::MotionCommand approach;
  approach.kind = machine::MotionKind::MoveJ;
  approach.target.joints = std::vector<double>{0, 0, 0, 0, 0, 0};
  task.push_back(approach);
  machine::MotionCommand weld;
  weld.kind = machine::MotionKind::MoveL;
  weld.target.pose = Pose3D{expected_seam, Quaternion::identity()};
  weld.via = Pose3D{Vec3{0.492, 0.10, 1.40}, Quaternion::identity()};
  task.push_back(weld);

  const machine::MotionTask corrected = runtime::apply_seam_offset(task, offset);
  check(!corrected[0].target.pose.has_value(), "MoveJ joint target is left untouched");
  check(corrected[0].target.joints.has_value(), "MoveJ joints survive unchanged");
  check(vdist(corrected[1].target.pose->position_m, observed) < 1e-9,
        "MoveL target is corrected onto the observed seam");
  check(vdist(corrected[1].via->position_m, Vec3{0.500, 0.105, 1.40}) < 1e-9,
        "MoveC via point is shifted by the same offset");
}

}  // namespace

int main() {
  test_cloud_to_base();
  test_seam_offset_and_correction();

  if (failures != 0) {
    std::cerr << failures << " vision guidance test(s) failed\n";
    return 1;
  }
  std::cout << "vision guidance tests passed\n";
  return 0;
}
