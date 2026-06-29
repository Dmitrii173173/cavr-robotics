#include <cavr/replay/scheduler.hpp>
#include <cavr/replay/session_loader.hpp>

#include <filesystem>
#include <iomanip>
#include <iostream>

namespace {

void print_usage() {
  std::cout << "Usage: cavr-play <session.json>\n";
}

void print_event(const cavr::replay::ReplayEvent& event) {
  const double seconds = static_cast<double>(event.timestamp.nanoseconds()) / 1'000'000'000.0;
  std::cout << std::fixed << std::setprecision(3) << seconds << " s | pose "
            << event.sample.pose_index << " | " << event.sample.image_path.filename().string() << '\n';
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

  const auto timeline = cavr::replay::build_timeline(result.session);
  cavr::replay::ReplayCursor cursor(timeline);
  while (const auto* event = cursor.step_next()) {
    print_event(*event);
  }

  return 0;
}
