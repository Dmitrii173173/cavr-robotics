#pragma once

// Replays a sequence of Netpbm image files from disk as a CameraAdapter, pacing
// frames at a configured rate against poll()'s clock. Mirrors MockCamera's role
// (the reference CameraAdapter for tests/demos) but sources real pixel data
// instead of a synthetic pattern — useful for replaying a captured or
// hand-authored image sequence without a vendor SDK or an image-decoding
// dependency.

#include <cavr/adapter_sdk/camera_adapter.hpp>
#include <cavr/adapters/file_camera/netpbm.hpp>

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace cavr::adapters::file_camera {

namespace sdk = cavr::adapter_sdk;

class FileCameraAdapter final : public sdk::CameraAdapter {
 public:
  // frame_paths: image files (.pgm/.ppm) in playback order. frame_id: the logical
  // camera name stamped on each frame. fps: how often a new frame becomes due,
  // measured against the timestamps passed to poll() (not wall-clock time).
  FileCameraAdapter(std::vector<std::filesystem::path> frame_paths, std::string frame_id, double fps = 30.0)
      : paths_(std::move(frame_paths)), frame_id_(std::move(frame_id)), fps_(fps > 0.0 ? fps : 30.0) {}

  // Every .pgm/.ppm file directly under `dir`, sorted by filename — the natural
  // way to point this adapter at a captured or hand-authored image sequence.
  [[nodiscard]] static FileCameraAdapter from_directory(const std::filesystem::path& dir, std::string frame_id,
                                                        double fps = 30.0) {
    std::vector<std::filesystem::path> paths;
    if (std::filesystem::exists(dir)) {
      for (const auto& entry : std::filesystem::directory_iterator(dir)) {
        if (!entry.is_regular_file()) continue;
        const auto ext = entry.path().extension().string();
        if (ext == ".pgm" || ext == ".ppm") paths.push_back(entry.path());
      }
    }
    std::sort(paths.begin(), paths.end());
    return FileCameraAdapter(std::move(paths), std::move(frame_id), fps);
  }

  [[nodiscard]] bool open() override {
    open_ = true;
    next_index_ = 0;
    started_ = false;
    return true;
  }
  [[nodiscard]] bool is_open() const override { return open_; }
  void close() override { open_ = false; }

  [[nodiscard]] std::optional<sdk::CameraFrame> poll(core::Timestamp now) override {
    if (!open_ || next_index_ >= paths_.size()) return std::nullopt;

    const std::int64_t now_ns = now.nanoseconds();
    if (!started_) {
      start_ns_ = now_ns;
      started_ = true;
    }
    const double elapsed_s = static_cast<double>(now_ns - start_ns_) * 1e-9;
    if (elapsed_s * fps_ < static_cast<double>(next_index_)) return std::nullopt;  // next frame not due yet

    const NetpbmLoad loaded = read_netpbm(paths_[next_index_]);
    ++next_index_;
    if (!loaded.ok) return std::nullopt;  // a malformed frame is skipped, not fatal to the sequence

    sdk::CameraFrame frame;
    frame.timestamp = now;
    frame.frame_id = frame_id_;
    frame.width = loaded.image.width;
    frame.height = loaded.image.height;
    frame.encoding = loaded.image.encoding;
    frame.pixels = std::move(loaded.image.pixels);
    return frame;
  }

  [[nodiscard]] std::size_t frame_count() const noexcept { return paths_.size(); }
  [[nodiscard]] std::size_t frames_emitted() const noexcept { return next_index_; }

 private:
  std::vector<std::filesystem::path> paths_;
  std::string frame_id_;
  double fps_;
  bool open_{false};
  bool started_{false};
  std::int64_t start_ns_{0};
  std::size_t next_index_{0};
};

}  // namespace cavr::adapters::file_camera
