#pragma once

// One camera image captured at an instant, time-aligned with robot telemetry so a
// session can carry a synchronized robot + vision stream. Pixel data is the heavy
// payload that belongs in the authoritative recording (MCAP) — never in a
// lightweight metadata catalog.

#include <cavr/core/time.hpp>

#include <cstdint>
#include <string>
#include <vector>

namespace cavr::adapter_sdk {

struct CameraFrame final {
  core::Timestamp timestamp{core::Timestamp::zero()};
  std::string frame_id;              // logical camera, e.g. "weld_cam"
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::string encoding;             // pixel layout, e.g. "mono8", "rgb8"
  std::vector<std::uint8_t> pixels;  // row-major, defined by `encoding`

  [[nodiscard]] bool empty() const noexcept { return pixels.empty(); }
  [[nodiscard]] std::size_t byte_size() const noexcept { return pixels.size(); }
};

}  // namespace cavr::adapter_sdk
