#pragma once

#include <cavr/core/geometry.hpp>
#include <cavr/core/time.hpp>

#include <filesystem>
#include <string>
#include <vector>

namespace cavr::replay {

struct PoseSample final {
  core::Timestamp timestamp;
  core::Pose3D pose;
};

struct ImageSample final {
  core::Timestamp timestamp;
  std::filesystem::path path;
};

struct ReplaySample final {
  core::Timestamp timestamp;
  std::size_t pose_index{};
  core::Pose3D pose;
  std::filesystem::path image_path;
};

struct SessionManifest final {
  std::filesystem::path manifest_path;
  std::filesystem::path poses_csv_path;
  std::filesystem::path images_directory;
  std::vector<PoseSample> poses;
  std::vector<ImageSample> images;
};

struct LoadedSession final {
  SessionManifest manifest;
  std::vector<ReplaySample> replay_samples;
};

struct LoadResult final {
  LoadedSession session;
  std::vector<std::string> errors;

  [[nodiscard]] bool ok() const noexcept {
    return errors.empty();
  }
};

}  // namespace cavr::replay
