// Calibration data model: the pinhole+distortion projection, the hand-eye frame
// algebra, and JSON round-trips. All pure maths — no camera, no robot — so the
// "Calibration-Aware" core can be asserted in isolation before any vision input.

#include <cavr/calibration/calibration_io.hpp>
#include <cavr/calibration/camera_intrinsics.hpp>
#include <cavr/calibration/hand_eye.hpp>

#include <cavr/machine/frames.hpp>

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

namespace cal = cavr::calibration;
namespace machine = cavr::machine;
using cavr::core::Pose3D;
using cavr::core::Quaternion;
using cavr::core::Vec3;

double vdist(const Vec3& a, const Vec3& b) {
  return std::sqrt((a.x_m - b.x_m) * (a.x_m - b.x_m) + (a.y_m - b.y_m) * (a.y_m - b.y_m) +
                   (a.z_m - b.z_m) * (a.z_m - b.z_m));
}

cal::CameraIntrinsics vga() {
  cal::CameraIntrinsics in;
  in.width = 640;
  in.height = 480;
  in.fx = 600.0;
  in.fy = 600.0;
  in.cx = 320.0;
  in.cy = 240.0;
  return in;
}

// A pinhole (no distortion) projects and unprojects as exact inverses.
void test_pinhole_roundtrip() {
  const auto in = vga();
  check(in.valid(), "vga intrinsics are valid");

  const Vec3 p{0.10, -0.05, 0.5};  // metres, in front of the camera
  const auto pix = cal::project(in, p);
  check(pix.has_value(), "point in front of the camera projects");
  // fx*x/z + cx = 600*0.2 + 320 = 440; fy*y/z + cy = 600*(-0.1) + 240 = 180
  check(std::abs(pix->u - 440.0) < 1e-9 && std::abs(pix->v - 180.0) < 1e-9,
        "pinhole projection matches the closed form");

  const Vec3 ray = cal::unproject(in, *pix);
  const Vec3 recovered{ray.x_m * p.z_m, ray.y_m * p.z_m, ray.z_m * p.z_m};
  check(vdist(recovered, p) < 1e-9, "unproject·depth recovers the original point");
}

// With distortion, project∘unproject is still identity in pixel space (the
// iterative undistortion inverts the distortion model).
void test_distortion_roundtrip() {
  auto in = vga();
  in.k1 = -0.28;
  in.k2 = 0.10;
  in.p1 = 0.001;
  in.p2 = -0.0005;

  const cal::Pixel observed{500.0, 130.0};
  const Vec3 ray = cal::unproject(in, observed);
  const auto reprojected = cal::project(in, ray);  // ray already has Z=1
  check(reprojected.has_value(), "undistorted ray reprojects");
  check(std::abs(reprojected->u - observed.u) < 1e-6 &&
            std::abs(reprojected->v - observed.v) < 1e-6,
        "project∘unproject is identity even with distortion");
}

// Points on or behind the image plane have no projection.
void test_behind_camera() {
  const auto in = vga();
  check(!cal::project(in, Vec3{0.1, 0.1, 0.0}).has_value(), "Z=0 does not project");
  check(!cal::project(in, Vec3{0.1, 0.1, -0.3}).has_value(), "Z<0 does not project");
}

// Reprojection error is zero for a consistent point and equals the pixel offset
// otherwise.
void test_reprojection_error() {
  const auto in = vga();
  const Vec3 p{0.10, -0.05, 0.5};
  const auto pix = cal::project(in, p);
  check(pix.has_value(), "reprojection test point projects");

  const auto zero = cal::reprojection_error(in, p, *pix);
  check(zero.has_value() && *zero < 1e-9, "self-consistent reprojection error is zero");

  const cal::Pixel off{pix->u + 3.0, pix->v + 4.0};  // 3-4-5 triangle
  const auto err = cal::reprojection_error(in, p, off);
  check(err.has_value() && std::abs(*err - 5.0) < 1e-9, "reprojection error is the pixel distance");
}

// Eye-in-hand: the camera rides the flange, so its base pose composes the flange
// pose with the fixed camera-in-flange transform, and a base point maps back to
// the camera frame consistently.
void test_hand_eye_eye_in_hand() {
  cal::HandEyeCalibration he;
  he.type = cal::HandEyeType::EyeInHand;
  he.camera_frame = "weld_cam";
  he.transform = Pose3D{Vec3{0.05, 0.0, 0.08}, Quaternion::identity()};

  const Pose3D flange{Vec3{0.6, 0.1, 0.9}, Quaternion::identity()};
  const Pose3D cam = cal::camera_in_base(he, flange);
  check(vdist(cam.position_m, Vec3{0.65, 0.1, 0.98}) < 1e-9,
        "eye-in-hand camera pose = flange ∘ camera-in-flange");

  // A point sitting 0.5 m in front of the camera (its +Z), expressed in base,
  // must come back to (0,0,0.5) in the camera frame.
  const Vec3 in_cam{0.0, 0.0, 0.5};
  const Vec3 in_base = machine::compose(cam, Pose3D{in_cam, Quaternion::identity()}).position_m;
  const Vec3 back = cal::point_base_to_camera(he, flange, in_base);
  check(vdist(back, in_cam) < 1e-9, "base→camera inverts camera→base for eye-in-hand");
}

// Eye-to-hand: the camera is fixed in the cell, so the flange pose is irrelevant.
void test_hand_eye_eye_to_hand() {
  cal::HandEyeCalibration he;
  he.type = cal::HandEyeType::EyeToHand;
  he.camera_frame = "cell_cam";
  he.transform = Pose3D{Vec3{1.0, 0.0, 1.5}, Quaternion::identity()};

  const Pose3D a{Vec3{0.6, 0.1, 0.9}, Quaternion::identity()};
  const Pose3D b{Vec3{-0.2, 0.4, 0.3}, Quaternion::identity()};
  check(vdist(cal::camera_in_base(he, a).position_m, cal::camera_in_base(he, b).position_m) < 1e-12,
        "eye-to-hand camera pose ignores the flange pose");
}

void test_json_roundtrip() {
  auto in = vga();
  in.k1 = -0.28;
  in.k2 = 0.10;
  in.p1 = 0.001;
  in.p2 = -0.0005;
  in.k3 = 0.002;
  const auto in2 = cal::intrinsics_from_string(cal::intrinsics_to_string(in));
  check(in2.has_value(), "intrinsics JSON parses");
  check(in2->width == in.width && in2->height == in.height &&
            std::abs(in2->fx - in.fx) < 1e-12 && std::abs(in2->cx - in.cx) < 1e-12 &&
            std::abs(in2->k1 - in.k1) < 1e-12 && std::abs(in2->p2 - in.p2) < 1e-12 &&
            std::abs(in2->k3 - in.k3) < 1e-12,
        "intrinsics survive a JSON round-trip");

  cal::HandEyeCalibration he;
  he.type = cal::HandEyeType::EyeToHand;
  he.camera_frame = "weld_cam";
  he.transform = Pose3D{Vec3{0.05, -0.02, 0.08}, Quaternion::identity()};
  he.method = "tsai-lenz";
  he.residual_position_m = 0.0012;
  he.residual_rotation_rad = 0.003;
  he.version = 2;
  he.calibrated_at_ns = 1720000000000000000LL;
  const auto he2 = cal::hand_eye_from_string(cal::hand_eye_to_string(he));
  check(he2.has_value(), "hand-eye JSON parses");
  check(he2->type == he.type && he2->camera_frame == he.camera_frame &&
            he2->method == he.method && he2->version == he.version &&
            he2->calibrated_at_ns == he.calibrated_at_ns &&
            vdist(he2->transform.position_m, he.transform.position_m) < 1e-12 &&
            std::abs(he2->residual_position_m - he.residual_position_m) < 1e-12,
        "hand-eye survives a JSON round-trip");
}

}  // namespace

int main() {
  test_pinhole_roundtrip();
  test_distortion_roundtrip();
  test_behind_camera();
  test_reprojection_error();
  test_hand_eye_eye_in_hand();
  test_hand_eye_eye_to_hand();
  test_json_roundtrip();

  if (failures != 0) {
    std::cerr << failures << " calibration test(s) failed\n";
    return 1;
  }
  std::cout << "calibration tests passed\n";
  return 0;
}
