#pragma once

// Pinhole camera intrinsics with Brown-Conrady (plumb-bob) distortion — the "how
// a 3D point becomes a pixel" half of calibration. Dependency-free maths over
// core::Vec3 so it works without a vendor SDK; a real intrinsics calibration
// routine (checkerboard, etc.) can fill these in later, but the projection model
// they feed is here now and testable.
//
// Convention: the camera looks down +Z in its own frame, X right, Y down (the
// usual computer-vision optical frame). Focal length and principal point are in
// pixels; distortion coefficients are dimensionless.

#include <cavr/core/geometry.hpp>

#include <cmath>
#include <cstdint>
#include <optional>

namespace cavr::calibration {

// A pixel coordinate (u across, v down), sub-pixel precision.
struct Pixel final {
  double u{0.0};
  double v{0.0};
};

struct CameraIntrinsics final {
  std::uint32_t width{0};
  std::uint32_t height{0};
  double fx{0.0};  // focal length x, pixels
  double fy{0.0};  // focal length y, pixels
  double cx{0.0};  // principal point x, pixels
  double cy{0.0};  // principal point y, pixels

  // Brown-Conrady: radial k1,k2,k3 and tangential p1,p2. Zero = pinhole.
  double k1{0.0};
  double k2{0.0};
  double k3{0.0};
  double p1{0.0};
  double p2{0.0};

  [[nodiscard]] bool valid() const noexcept {
    return width > 0 && height > 0 && fx > 0.0 && fy > 0.0 &&
           std::isfinite(cx) && std::isfinite(cy);
  }
};

namespace detail {

// Apply Brown-Conrady distortion to a normalised image-plane point (x,y) =
// (X/Z, Y/Z), returning the distorted normalised coordinates.
inline void distort(const CameraIntrinsics& in, double x, double y, double& xd, double& yd) {
  const double r2 = x * x + y * y;
  const double radial = 1.0 + r2 * (in.k1 + r2 * (in.k2 + r2 * in.k3));
  const double x_tan = 2.0 * in.p1 * x * y + in.p2 * (r2 + 2.0 * x * x);
  const double y_tan = in.p1 * (r2 + 2.0 * y * y) + 2.0 * in.p2 * x * y;
  xd = x * radial + x_tan;
  yd = y * radial + y_tan;
}

}  // namespace detail

// Project a point expressed in the CAMERA frame (metres, +Z forward) to a pixel.
// Returns nullopt when the point is on or behind the image plane (Z <= 0), which
// has no valid projection.
[[nodiscard]] inline std::optional<Pixel> project(const CameraIntrinsics& in, const core::Vec3& point_cam) {
  if (point_cam.z_m <= 0.0) return std::nullopt;
  const double x = point_cam.x_m / point_cam.z_m;
  const double y = point_cam.y_m / point_cam.z_m;
  double xd = 0.0, yd = 0.0;
  detail::distort(in, x, y, xd, yd);
  return Pixel{in.fx * xd + in.cx, in.fy * yd + in.cy};
}

// Unproject a pixel to a ray direction in the CAMERA frame, normalised so Z = 1
// (scale by depth to recover a 3D point). Distortion is removed iteratively; with
// zero coefficients this is the exact pinhole inverse of project().
[[nodiscard]] inline core::Vec3 unproject(const CameraIntrinsics& in, const Pixel& pixel) {
  const double xd = (pixel.u - in.cx) / in.fx;
  const double yd = (pixel.v - in.cy) / in.fy;

  // Newton-free fixed-point undistortion: invert the distortion by iterating
  // x = (xd - tangential) / radial. Converges quickly for realistic lenses.
  double x = xd, y = yd;
  for (int iter = 0; iter < 20; ++iter) {
    const double r2 = x * x + y * y;
    const double radial = 1.0 + r2 * (in.k1 + r2 * (in.k2 + r2 * in.k3));
    const double x_tan = 2.0 * in.p1 * x * y + in.p2 * (r2 + 2.0 * x * x);
    const double y_tan = in.p1 * (r2 + 2.0 * y * y) + 2.0 * in.p2 * x * y;
    x = (xd - x_tan) / radial;
    y = (yd - y_tan) / radial;
  }
  return core::Vec3{x, y, 1.0};
}

// Reprojection error (pixels): the distance between where a camera-frame point
// lands and where it was observed. The core quality metric for any calibration.
// Returns nullopt when the point does not project (behind the camera).
[[nodiscard]] inline std::optional<double> reprojection_error(const CameraIntrinsics& in,
                                                              const core::Vec3& point_cam,
                                                              const Pixel& observed) {
  const auto projected = project(in, point_cam);
  if (!projected) return std::nullopt;
  const double du = projected->u - observed.u;
  const double dv = projected->v - observed.v;
  return std::sqrt(du * du + dv * dv);
}

}  // namespace cavr::calibration
