#include <cavr/core/geometry.hpp>
#include <cavr/core/time.hpp>

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

void test_timestamp_and_duration() {
  using cavr::core::Duration;
  using cavr::core::Timestamp;

  const auto start = Timestamp::from_nanoseconds(1'000'000'000);
  const auto step = Duration::from_nanoseconds(100'000'000);
  const auto next = start + step;

  check(start.nanoseconds() == 1'000'000'000, "timestamp stores nanoseconds");
  check(step.to_chrono().count() == 100'000'000, "duration converts to chrono nanoseconds");
  check(next.nanoseconds() == 1'100'000'000, "timestamp plus duration");
  check((next - start).nanoseconds() == 100'000'000, "timestamp difference is duration");
}

void test_quaternion_validation() {
  using cavr::core::Quaternion;

  const auto valid = Quaternion::from_xyzw(0.0, 0.0, std::sqrt(0.5), std::sqrt(0.5));
  check(valid.has_value(), "unit quaternion is accepted");
  check(valid && std::abs(valid->norm_squared() - 1.0) < 1.0e-12, "accepted quaternion stays normalized");

  const auto zero = Quaternion::from_xyzw(0.0, 0.0, 0.0, 0.0);
  check(!zero.has_value(), "zero quaternion is rejected");

  const auto non_unit = Quaternion::from_xyzw(0.0, 0.0, 0.0, 2.0);
  check(!non_unit.has_value(), "non-unit quaternion is rejected");
}

void test_pose_construction() {
  const cavr::core::Pose3D pose{
      cavr::core::Vec3{0.5, 0.25, 1.2},
      cavr::core::Quaternion::identity(),
  };

  check(pose.position_m.x_m == 0.5, "pose position uses meters");
  check(pose.orientation.w() == 1.0, "pose has normalized identity orientation");
}

}  // namespace

int main() {
  test_timestamp_and_duration();
  test_quaternion_validation();
  test_pose_construction();

  if (failures != 0) {
    std::cerr << failures << " core domain type test(s) failed" << '\n';
    return 1;
  }

  return 0;
}
