// Dependency-free PNG decoding: decode a real zlib-compressed PNG (dynamic-Huffman
// IDAT, produced by a standard encoder) to validate the from-scratch inflater, then
// round-trip mono8/rgb8 images through the stored-block writer + decoder, and finally
// replay a .png through FileCameraAdapter to confirm the adapter picks the decoder by
// extension.

#include <cavr/adapters/file_camera/file_camera_adapter.hpp>
#include <cavr/adapters/file_camera/png.hpp>

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

// An 8x8 RGB gradient PNG compressed with zlib level 9 (a dynamic-Huffman DEFLATE
// block — the hardest inflate path), and the raw RGB bytes it must decode to.
// Generated once with Python's zlib; embedded so the test needs no external tool.
const unsigned char kPngRgb8x8[] = {
    137,80,78,71,13,10,26,10,0,0,0,13,73,72,68,82,0,0,0,8,0,0,0,8,8,2,0,0,0,75,109,41,220,0,0,0,
    132,73,68,65,84,120,218,13,200,209,0,4,49,16,3,208,32,44,194,32,44,66,16,138,16,132,34,4,161,8,65,
    88,132,65,56,132,65,185,190,207,7,0,133,135,40,225,53,24,172,134,6,251,246,83,85,172,87,69,215,74,
    169,107,79,25,224,237,151,164,184,76,133,187,233,225,1,244,150,110,47,73,214,142,220,58,163,0,102,
    121,209,183,183,237,248,180,51,254,128,172,138,152,173,220,62,73,58,223,164,129,86,245,102,91,125,
    220,183,191,238,158,254,1,179,107,204,57,154,120,190,204,237,223,204,252,1,238,63,84,1,164,168,169,
    206,0,0,0,0,73,69,78,68,174,66,96,130};

const unsigned char kExpectedRgb[] = {
    0,0,0,32,0,16,64,0,32,96,0,48,128,0,64,160,0,80,192,0,96,224,0,112,0,32,16,32,32,32,64,32,48,96,32,
    64,128,32,80,160,32,96,192,32,112,224,32,128,0,64,32,32,64,48,64,64,64,96,64,80,128,64,96,160,64,112,
    192,64,128,224,64,144,0,96,48,32,96,64,64,96,80,96,96,96,128,96,112,160,96,128,192,96,144,224,96,160,
    0,128,64,32,128,80,64,128,96,96,128,112,128,128,128,160,128,144,192,128,160,224,128,176,0,160,80,32,
    160,96,64,160,112,96,160,128,128,160,144,160,160,160,192,160,176,224,160,192,0,192,96,32,192,112,64,
    192,128,96,192,144,128,192,160,160,192,176,192,192,192,224,192,208,0,224,112,32,224,128,64,224,144,96,
    224,160,128,224,176,160,224,192,192,224,208,224,224,224};

// Decode the embedded compressed PNG: exercises the full inflater (dynamic Huffman
// + LZ77 back-references) and unfiltering.
void test_decode_compressed() {
  const auto dir = temp_dir("cavr_png_compressed");
  const auto path = dir / "gradient.png";
  {
    std::ofstream os(path, std::ios::binary);
    os.write(reinterpret_cast<const char*>(kPngRgb8x8), sizeof(kPngRgb8x8));
  }

  const auto loaded = file_camera::read_png(path);
  check(loaded.ok, "compressed PNG decodes");
  check(loaded.image.width == 8 && loaded.image.height == 8, "PNG dimensions from IHDR");
  check(loaded.image.encoding == "rgb8", "truecolor PNG is rgb8");
  const std::vector<std::uint8_t> expected(kExpectedRgb, kExpectedRgb + sizeof(kExpectedRgb));
  check(loaded.image.pixels == expected, "decoded pixels match the source gradient exactly");

  std::filesystem::remove_all(dir);
}

// write_png (stored DEFLATE blocks) -> read_png must round-trip both encodings.
void test_writer_roundtrip() {
  const auto dir = temp_dir("cavr_png_roundtrip");

  const auto gray_path = dir / "gray.png";
  const std::vector<std::uint8_t> gray = {10, 20, 30, 40, 50, 60};  // 3x2 mono8
  check(file_camera::write_png(gray_path, 3, 2, 1, gray), "mono8 PNG writes");
  const auto gray_loaded = file_camera::read_png(gray_path);
  check(gray_loaded.ok && gray_loaded.image.encoding == "mono8", "mono8 PNG reads back");
  check(gray_loaded.image.width == 3 && gray_loaded.image.height == 2, "mono8 dimensions round-trip");
  check(gray_loaded.image.pixels == gray, "mono8 pixels round-trip exactly");

  const auto rgb_path = dir / "rgb.png";
  const std::vector<std::uint8_t> rgb = {255, 0, 0, 0, 255, 0, 0, 0, 255, 255, 255, 255};  // 2x2 rgb8
  check(file_camera::write_png(rgb_path, 2, 2, 3, rgb), "rgb8 PNG writes");
  const auto rgb_loaded = file_camera::read_png(rgb_path);
  check(rgb_loaded.ok && rgb_loaded.image.encoding == "rgb8", "rgb8 PNG reads back");
  check(rgb_loaded.image.pixels == rgb, "rgb8 pixels round-trip exactly");

  std::filesystem::remove_all(dir);
}

// A non-PNG file is rejected cleanly (skipped, not a crash).
void test_rejects_garbage() {
  const auto dir = temp_dir("cavr_png_bad");
  const auto path = dir / "bad.png";
  {
    std::ofstream os(path, std::ios::binary);
    os << "not a png at all";
  }
  const auto loaded = file_camera::read_png(path);
  check(!loaded.ok, "garbage is rejected with an error, not decoded");
  std::filesystem::remove_all(dir);
}

// The adapter picks the PNG decoder by extension and emits the frame.
void test_adapter_replays_png() {
  const auto dir = temp_dir("cavr_png_adapter");
  const std::vector<std::uint8_t> rgb = {1, 2, 3, 4, 5, 6};  // 2x1 rgb8
  check(file_camera::write_png(dir / "frame.png", 2, 1, 3, rgb), "adapter fixture written");

  auto camera = file_camera::FileCameraAdapter::from_directory(dir, "weld_cam", 30.0);
  check(camera.frame_count() == 1, "adapter found the .png frame");
  check(camera.open(), "adapter opens");

  const auto frame = camera.poll(cavr::core::Timestamp::from_nanoseconds(0));
  check(frame.has_value(), "adapter emits the PNG frame");
  check(frame->encoding == "rgb8" && frame->width == 2 && frame->height == 1,
        "adapter frame carries the decoded geometry");
  check(frame->pixels == rgb, "adapter frame pixels match the source");

  std::filesystem::remove_all(dir);
}

}  // namespace

int main() {
  test_decode_compressed();
  test_writer_roundtrip();
  test_rejects_garbage();
  test_adapter_replays_png();

  if (failures != 0) {
    std::cerr << failures << " PNG test(s) failed\n";
    return 1;
  }
  std::cout << "file camera PNG tests passed\n";
  return 0;
}
