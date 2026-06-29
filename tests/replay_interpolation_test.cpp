#include <cavr/replay/interpolation.hpp>

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

bool near(double lhs, double rhs, double tolerance = 1.0e-12) {
  return std::abs(lhs - rhs) <= tolerance;
}

cavr::replay::PoseSample sample(std::int64_t timestamp_ns, double x_m, double y_m, double z_m) {
  return cavr::replay::PoseSample{
      cavr::core::Timestamp::from_nanoseconds(timestamp_ns),
      cavr::core::Pose3D{
          cavr::core::Vec3{x_m, y_m, z_m},
          cavr::core::Quaternion::identity(),
      },
  };
}

}  // namespace

int main() {
  const std::vector<cavr::replay::PoseSample> poses = {
      sample(0, 0.0, 0.0, 1.0),
      sample(100'000'000, 1.0, 2.0, 3.0),
      sample(200'000'000, 3.0, 4.0, 5.0),
  };

  const auto exact = cavr::replay::interpolate_pose_at(
      poses, cavr::core::Timestamp::from_nanoseconds(100'000'000));
  check(exact.has_value(), "exact timestamp returns a pose");
  check(exact && near(exact->position_m.x_m, 1.0), "exact timestamp returns exact x position");
  check(exact && near(exact->position_m.y_m, 2.0), "exact timestamp returns exact y position");

  const auto midpoint = cavr::replay::interpolate_pose_at(
      poses, cavr::core::Timestamp::from_nanoseconds(50'000'000));
  check(midpoint.has_value(), "midpoint timestamp returns a pose");
  check(midpoint && near(midpoint->position_m.x_m, 0.5), "midpoint interpolates x in meters");
  check(midpoint && near(midpoint->position_m.y_m, 1.0), "midpoint interpolates y in meters");
  check(midpoint && near(midpoint->position_m.z_m, 2.0), "midpoint interpolates z in meters");
  check(midpoint && near(midpoint->orientation.norm_squared(), 1.0), "interpolated orientation is normalized");

  const auto before = cavr::replay::interpolate_pose_at(
      poses, cavr::core::Timestamp::from_nanoseconds(-1));
  check(!before.has_value(), "timestamp before range returns empty");

  const auto after = cavr::replay::interpolate_pose_at(
      poses, cavr::core::Timestamp::from_nanoseconds(300'000'000));
  check(!after.has_value(), "timestamp after range returns empty");

  const auto duplicate_time = cavr::replay::interpolate_pose(
      sample(100'000'000, 0.0, 0.0, 0.0),
      sample(100'000'000, 1.0, 1.0, 1.0),
      cavr::core::Timestamp::from_nanoseconds(100'000'000));
  check(!duplicate_time.has_value(), "duplicate timestamps cannot be interpolated");

  const std::vector<cavr::replay::PoseSample> single_pose = {sample(42, 0.1, 0.2, 0.3)};
  const auto single_exact = cavr::replay::interpolate_pose_at(
      single_pose, cavr::core::Timestamp::from_nanoseconds(42));
  const auto single_miss = cavr::replay::interpolate_pose_at(
      single_pose, cavr::core::Timestamp::from_nanoseconds(43));
  check(single_exact.has_value(), "single exact pose returns value");
  check(!single_miss.has_value(), "single non-exact pose returns empty");

  return failures == 0 ? 0 : 1;
}
