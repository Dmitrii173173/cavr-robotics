#pragma once

// Deterministic camera used by tests, examples and the Studio demo. Each poll
// emits a small synthetic grayscale frame whose pattern varies with time, so
// frames are distinguishable and reproducible. The reference CameraAdapter
// implementation, mirroring mock_robot's MockController.

#include <cavr/adapter_sdk/camera_adapter.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>

namespace cavr::adapters::mock_camera {

namespace sdk = cavr::adapter_sdk;

class MockCamera final : public sdk::CameraAdapter {
 public:
  explicit MockCamera(std::uint32_t width = 8, std::uint32_t height = 8,
                      std::string frame_id = "weld_cam")
      : width_(width), height_(height), frame_id_(std::move(frame_id)) {}

  [[nodiscard]] bool open() override {
    open_ = true;
    return true;
  }
  [[nodiscard]] bool is_open() const override { return open_; }
  void close() override { open_ = false; }

  [[nodiscard]] std::optional<sdk::CameraFrame> poll(cavr::core::Timestamp now) override {
    if (!open_) return std::nullopt;

    sdk::CameraFrame frame;
    frame.timestamp = now;
    frame.frame_id = frame_id_;
    frame.width = width_;
    frame.height = height_;
    frame.encoding = "mono8";

    // A reproducible ramp seeded by the millisecond timestamp.
    const auto seed = static_cast<std::uint8_t>((now.nanoseconds() / 1'000'000) & 0xFF);
    frame.pixels.resize(static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_));
    for (std::size_t i = 0; i < frame.pixels.size(); ++i) {
      frame.pixels[i] = static_cast<std::uint8_t>((seed + i) & 0xFF);
    }

    ++count_;
    return frame;
  }

  // A small synthetic scan: a grid of points on a plane ~0.5 m in front of the
  // camera, gently rippled and shifted by time so successive clouds differ, with a
  // grayscale color per point. Deterministic, so tests can assert it verbatim.
  [[nodiscard]] std::optional<sdk::PointCloud> poll_point_cloud(cavr::core::Timestamp now) override {
    if (!open_) return std::nullopt;

    sdk::PointCloud cloud;
    cloud.timestamp = now;
    cloud.frame_id = frame_id_;

    const double phase = static_cast<double>((now.nanoseconds() / 1'000'000) & 0xFF) * 0.01;
    for (std::uint32_t v = 0; v < cloud_rows_; ++v) {
      for (std::uint32_t u = 0; u < cloud_cols_; ++u) {
        const double x = (static_cast<double>(u) / cloud_cols_ - 0.5) * 0.2;  // ±0.1 m
        const double y = (static_cast<double>(v) / cloud_rows_ - 0.5) * 0.2;
        const double z = 0.5 + 0.01 * std::sin(phase + u + v);                // depth, +Z forward
        cloud.points.push_back(cavr::core::Vec3{x, y, z});
        const auto g = static_cast<std::uint8_t>((u * 16 + v * 16) & 0xFF);
        cloud.colors.push_back(sdk::PointColor{g, g, g});
      }
    }

    ++cloud_count_;
    return cloud;
  }

  [[nodiscard]] std::size_t frame_count() const noexcept { return count_; }
  [[nodiscard]] std::size_t point_cloud_count() const noexcept { return cloud_count_; }

 private:
  std::uint32_t width_;
  std::uint32_t height_;
  std::string frame_id_;
  bool open_{false};
  std::size_t count_{0};
  std::size_t cloud_count_{0};
  std::uint32_t cloud_rows_{4};
  std::uint32_t cloud_cols_{4};
};

}  // namespace cavr::adapters::mock_camera
