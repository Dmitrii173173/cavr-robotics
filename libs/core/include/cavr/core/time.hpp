#pragma once

#include <chrono>
#include <cstdint>

namespace cavr::core {

class Duration final {
 public:
  using rep = std::int64_t;

  [[nodiscard]] static constexpr Duration from_nanoseconds(rep value) noexcept {
    return Duration(value);
  }

  [[nodiscard]] static constexpr Duration zero() noexcept {
    return Duration(0);
  }

  [[nodiscard]] constexpr rep nanoseconds() const noexcept {
    return nanoseconds_;
  }

  [[nodiscard]] constexpr std::chrono::nanoseconds to_chrono() const noexcept {
    return std::chrono::nanoseconds(nanoseconds_);
  }

 private:
  explicit constexpr Duration(rep nanoseconds) noexcept : nanoseconds_(nanoseconds) {}

  rep nanoseconds_{};
};

class Timestamp final {
 public:
  using rep = std::int64_t;

  [[nodiscard]] static constexpr Timestamp from_nanoseconds(rep value) noexcept {
    return Timestamp(value);
  }

  [[nodiscard]] static constexpr Timestamp zero() noexcept {
    return Timestamp(0);
  }

  [[nodiscard]] constexpr rep nanoseconds() const noexcept {
    return nanoseconds_;
  }

  [[nodiscard]] constexpr std::chrono::nanoseconds to_chrono() const noexcept {
    return std::chrono::nanoseconds(nanoseconds_);
  }

 private:
  explicit constexpr Timestamp(rep nanoseconds) noexcept : nanoseconds_(nanoseconds) {}

  rep nanoseconds_{};
};

[[nodiscard]] constexpr bool operator==(Duration lhs, Duration rhs) noexcept {
  return lhs.nanoseconds() == rhs.nanoseconds();
}

[[nodiscard]] constexpr bool operator!=(Duration lhs, Duration rhs) noexcept {
  return !(lhs == rhs);
}

[[nodiscard]] constexpr bool operator<(Duration lhs, Duration rhs) noexcept {
  return lhs.nanoseconds() < rhs.nanoseconds();
}

[[nodiscard]] constexpr bool operator==(Timestamp lhs, Timestamp rhs) noexcept {
  return lhs.nanoseconds() == rhs.nanoseconds();
}

[[nodiscard]] constexpr bool operator!=(Timestamp lhs, Timestamp rhs) noexcept {
  return !(lhs == rhs);
}

[[nodiscard]] constexpr bool operator<(Timestamp lhs, Timestamp rhs) noexcept {
  return lhs.nanoseconds() < rhs.nanoseconds();
}

[[nodiscard]] constexpr Timestamp operator+(Timestamp timestamp, Duration duration) noexcept {
  return Timestamp::from_nanoseconds(timestamp.nanoseconds() + duration.nanoseconds());
}

[[nodiscard]] constexpr Timestamp operator-(Timestamp timestamp, Duration duration) noexcept {
  return Timestamp::from_nanoseconds(timestamp.nanoseconds() - duration.nanoseconds());
}

[[nodiscard]] constexpr Duration operator-(Timestamp lhs, Timestamp rhs) noexcept {
  return Duration::from_nanoseconds(lhs.nanoseconds() - rhs.nanoseconds());
}

}  // namespace cavr::core
