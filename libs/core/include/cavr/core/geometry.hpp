#pragma once

#include <cmath>
#include <optional>

namespace cavr::core {

struct Vec3 final {
  double x_m{};
  double y_m{};
  double z_m{};
};

class Quaternion final {
 public:
  [[nodiscard]] static constexpr Quaternion identity() noexcept {
    return Quaternion(0.0, 0.0, 0.0, 1.0);
  }

  [[nodiscard]] static std::optional<Quaternion> from_xyzw(
      double x, double y, double z, double w, double tolerance = 1.0e-9) noexcept {
    const double norm = x * x + y * y + z * z + w * w;
    if (!std::isfinite(norm) || std::abs(norm - 1.0) > tolerance) {
      return std::nullopt;
    }
    return Quaternion(x, y, z, w);
  }

  [[nodiscard]] constexpr double x() const noexcept { return x_; }
  [[nodiscard]] constexpr double y() const noexcept { return y_; }
  [[nodiscard]] constexpr double z() const noexcept { return z_; }
  [[nodiscard]] constexpr double w() const noexcept { return w_; }

  [[nodiscard]] constexpr double norm_squared() const noexcept {
    return x_ * x_ + y_ * y_ + z_ * z_ + w_ * w_;
  }

 private:
  constexpr Quaternion(double x, double y, double z, double w) noexcept
      : x_(x), y_(y), z_(z), w_(w) {}

  double x_{};
  double y_{};
  double z_{};
  double w_{1.0};
};

struct Pose3D final {
  Vec3 position_m{};
  Quaternion orientation = Quaternion::identity();
};

}  // namespace cavr::core
