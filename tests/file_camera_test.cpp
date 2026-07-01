// FileCameraAdapter: round-trips synthetic Netpbm fixtures through the reader/
// writer, then replays them as a CameraAdapter and checks that poll() paces
// frames against its clock argument (not wall-clock time) and yields them in
// filename order, matching MockCamera's contract.

#include <cavr/adapters/file_camera/file_camera_adapter.hpp>
#include <cavr/adapters/file_camera/netpbm.hpp>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string_view>
#include <vector>

namespace {

int failures = 0;

void check(bool condition, std::string_view message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
    ++failures;
  }
}

namespace file_camera = cavr::adapters::file_camera;

std::filesystem::path temp_dir(std::string_view name) {
  const auto dir = std::filesystem::temp_directory_path() / name;
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  return dir;
}

void test_pgm_round_trip() {
  const auto dir = temp_dir("cavr_file_camera_pgm");
  const auto path = dir / "frame.pgm";
  const std::vector<std::uint8_t> pixels = {10, 20, 30, 40};  // 2x2 mono8
  check(file_camera::write_pgm(path, 2, 2, pixels), "pgm writes");

  const auto loaded = file_camera::read_netpbm(path);
  check(loaded.ok, "pgm reads back");
  check(loaded.image.width == 2 && loaded.image.height == 2, "pgm dimensions round-trip");
  check(loaded.image.encoding == "mono8", "pgm encoding is mono8");
  check(loaded.image.pixels == pixels, "pgm pixel bytes round-trip exactly");

  std::filesystem::remove_all(dir);
}

void test_ppm_round_trip() {
  const auto dir = temp_dir("cavr_file_camera_ppm");
  const auto path = dir / "frame.ppm";
  const std::vector<std::uint8_t> pixels = {255, 0, 0, 0, 255, 0, 0, 0, 255, 255, 255, 255};  // 2x2 rgb8
  check(file_camera::write_ppm(path, 2, 2, pixels), "ppm writes");

  const auto loaded = file_camera::read_netpbm(path);
  check(loaded.ok, "ppm reads back");
  check(loaded.image.width == 2 && loaded.image.height == 2, "ppm dimensions round-trip");
  check(loaded.image.encoding == "rgb8", "ppm encoding is rgb8");
  check(loaded.image.pixels == pixels, "ppm pixel bytes round-trip exactly");

  std::filesystem::remove_all(dir);
}

void test_malformed_image_rejected() {
  const auto dir = temp_dir("cavr_file_camera_bad");
  const auto path = dir / "not_an_image.pgm";
  {
    std::ofstream out(path, std::ios::binary);
    out << "not a netpbm file at all";
  }
  const auto loaded = file_camera::read_netpbm(path);
  check(!loaded.ok, "garbage input is rejected, not crashed on");
  check(!loaded.error.empty(), "rejection carries an error message");

  std::filesystem::remove_all(dir);
}

void test_adapter_replays_in_order_paced_by_clock() {
  const auto dir = temp_dir("cavr_file_camera_sequence");
  // Three single-pixel frames, named so directory iteration order isn't already sorted.
  check(file_camera::write_pgm(dir / "b_frame.pgm", 1, 1, {20}), "frame b written");
  check(file_camera::write_pgm(dir / "a_frame.pgm", 1, 1, {10}), "frame a written");
  check(file_camera::write_pgm(dir / "c_frame.pgm", 1, 1, {30}), "frame c written");

  auto camera = file_camera::FileCameraAdapter::from_directory(dir, "test_cam", /*fps=*/10.0);
  check(camera.frame_count() == 3, "all three frames discovered");
  check(camera.open(), "camera opens");

  // At 10 fps, frame k is due at t = k / 10 s.
  const auto at = [](double seconds) {
    return cavr::core::Timestamp::from_nanoseconds(static_cast<std::int64_t>(seconds * 1e9));
  };

  const auto f0 = camera.poll(at(0.0));
  check(f0.has_value(), "first frame is due immediately");
  check(f0 && f0->pixels.size() == 1 && f0->pixels[0] == 10, "frames replay in filename order (a, not b)");
  check(f0 && f0->frame_id == "test_cam", "frame is stamped with the configured frame id");

  const auto not_yet = camera.poll(at(0.05));
  check(!not_yet.has_value(), "second frame is not due yet at t=0.05s");

  const auto f1 = camera.poll(at(0.10));
  check(f1.has_value() && f1->pixels[0] == 20, "second frame (b) is due at t=0.10s");

  const auto f2 = camera.poll(at(0.25));
  check(f2.has_value() && f2->pixels[0] == 30, "third frame (c) is due once its time has passed");

  check(camera.frames_emitted() == 3, "all frames emitted");
  const auto past_end = camera.poll(at(1.0));
  check(!past_end.has_value(), "polling past the last frame yields nothing");

  camera.close();
  check(!camera.is_open(), "camera reports closed");

  std::filesystem::remove_all(dir);
}

}  // namespace

int main() {
  test_pgm_round_trip();
  test_ppm_round_trip();
  test_malformed_image_rejected();
  test_adapter_replays_in_order_paced_by_clock();

  if (failures != 0) {
    std::cerr << failures << " file camera test(s) failed\n";
    return 1;
  }
  std::cout << "file camera tests passed\n";
  return 0;
}
