#include <cavr/replay/session_loader.hpp>

#include <cmath>
#include <filesystem>
#include <iostream>
#include <string_view>

#ifndef CAVR_TEST_DATA_DIR
#error "CAVR_TEST_DATA_DIR must be defined"
#endif

namespace {

int failures = 0;

void check(bool condition, std::string_view message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
    ++failures;
  }
}

}  // namespace

int main() {
  const auto manifest_path = std::filesystem::path(CAVR_TEST_DATA_DIR) / "demo_csv" / "session.json";
  const auto result = cavr::replay::load_session_manifest(manifest_path);

  if (!result.ok()) {
    for (const auto& error : result.errors) {
      std::cerr << "load error: " << error << '\n';
    }
    return 1;
  }

  check(result.session.manifest.poses.size() == 3, "loads three pose samples");
  check(result.session.manifest.images.size() == 3, "loads three image samples");
  check(result.session.replay_samples.size() == 3, "creates three synchronized replay samples");

  check(result.session.replay_samples[0].timestamp.nanoseconds() == 0, "first sample timestamp");
  check(result.session.replay_samples[1].timestamp.nanoseconds() == 100'000'000, "second sample timestamp");
  check(result.session.replay_samples[2].timestamp.nanoseconds() == 200'000'000, "third sample timestamp");

  check(result.session.replay_samples[0].pose_index == 0, "first pose index is deterministic");
  check(result.session.replay_samples[1].pose_index == 1, "second pose index is deterministic");
  check(result.session.replay_samples[2].pose_index == 2, "third pose index is deterministic");

  check(result.session.replay_samples[0].image_path.filename() == "frame_0000.png", "first image match");
  check(result.session.replay_samples[1].image_path.filename() == "frame_0001.png", "second image match");
  check(result.session.replay_samples[2].image_path.filename() == "frame_0002.png", "third image match");

  check(std::abs(result.session.replay_samples[2].pose.position_m.z_m - 0.820) < 1.0e-12,
        "pose position is loaded in meters");

  return failures == 0 ? 0 : 1;
}
