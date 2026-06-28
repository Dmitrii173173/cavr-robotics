#include <cavr/replay/session_loader.hpp>

#include <filesystem>
#include <iomanip>
#include <iostream>

namespace {

void print_usage() {
  std::cout << "Usage: cavr-play <session.json>\n";
}

}  // namespace

int main(int argc, char* argv[]) {
  if (argc != 2) {
    print_usage();
    return argc == 1 ? 0 : 2;
  }

  const std::filesystem::path manifest_path = argv[1];
  const auto result = cavr::replay::load_session_manifest(manifest_path);
  if (!result.ok()) {
    for (const auto& error : result.errors) {
      std::cerr << "error: " << error << '\n';
    }
    return 1;
  }

  for (const auto& sample : result.session.replay_samples) {
    const double seconds = static_cast<double>(sample.timestamp.nanoseconds()) / 1'000'000'000.0;
    std::cout << std::fixed << std::setprecision(3) << seconds << " s | pose "
              << sample.pose_index << " | " << sample.image_path.filename().string() << '\n';
  }

  return 0;
}
