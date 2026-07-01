#pragma once

// Minimal, dependency-free reader/writer for binary Netpbm images (PGM "P5" for
// 8-bit grayscale, PPM "P6" for 8-bit RGB) — the format FileCameraAdapter replays
// from disk. Chosen because it needs no image-decoding dependency (unlike
// PNG/JPEG): the pixel data is already raw bytes in the layout CameraFrame wants,
// behind a tiny plain-text header.
//
// Format (per the Netpbm spec): magic ("P5"/"P6"), whitespace, width, whitespace,
// height, whitespace, maxval, exactly one whitespace character, then raw binary
// pixel data. '#' starts a comment that runs to end of line; comments and extra
// whitespace may appear between header tokens. Only 8-bit images (maxval <= 255)
// are supported, matching the project's "mono8"/"rgb8" encodings.

#include <cctype>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <istream>
#include <string>
#include <vector>

namespace cavr::adapters::file_camera {

struct NetpbmImage final {
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::string encoding;  // "mono8" (P5) or "rgb8" (P6)
  std::vector<std::uint8_t> pixels;
};

struct NetpbmLoad final {
  NetpbmImage image;
  bool ok{false};
  std::string error;
};

namespace detail {

// Skips whitespace and '#' comments, then reads one whitespace-terminated token.
// Returns -1 on EOF or a malformed (non-numeric) token.
[[nodiscard]] inline int read_header_token(std::istream& in) {
  int c = in.get();
  while (c != EOF) {
    if (c == '#') {
      while (c != EOF && c != '\n') c = in.get();
    } else if (std::isspace(static_cast<unsigned char>(c))) {
      c = in.get();
    } else {
      break;
    }
  }
  if (c == EOF) return -1;

  std::string digits;
  while (c != EOF && !std::isspace(static_cast<unsigned char>(c))) {
    digits.push_back(static_cast<char>(c));
    c = in.get();
  }
  if (digits.empty()) return -1;
  for (char ch : digits) {
    if (!std::isdigit(static_cast<unsigned char>(ch))) return -1;
  }
  return std::stoi(digits);
}

}  // namespace detail

[[nodiscard]] inline NetpbmLoad read_netpbm(const std::filesystem::path& path) {
  NetpbmLoad result;
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    result.error = "Failed to open image: " + path.string();
    return result;
  }

  char magic[2] = {};
  in.read(magic, 2);
  int channels = 0;
  if (magic[0] == 'P' && magic[1] == '5') {
    channels = 1;
    result.image.encoding = "mono8";
  } else if (magic[0] == 'P' && magic[1] == '6') {
    channels = 3;
    result.image.encoding = "rgb8";
  } else {
    result.error = "Unsupported image format (expected binary Netpbm P5/P6): " + path.string();
    return result;
  }

  const int width = detail::read_header_token(in);
  const int height = detail::read_header_token(in);
  const int maxval = detail::read_header_token(in);
  if (width <= 0 || height <= 0 || maxval <= 0) {
    result.error = "Malformed Netpbm header: " + path.string();
    return result;
  }
  if (maxval > 255) {
    result.error = "Only 8-bit Netpbm images (maxval <= 255) are supported: " + path.string();
    return result;
  }

  result.image.width = static_cast<std::uint32_t>(width);
  result.image.height = static_cast<std::uint32_t>(height);
  const std::size_t byte_count =
      static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * static_cast<std::size_t>(channels);
  result.image.pixels.resize(byte_count);
  in.read(reinterpret_cast<char*>(result.image.pixels.data()), static_cast<std::streamsize>(byte_count));
  if (static_cast<std::size_t>(in.gcount()) != byte_count) {
    result.error = "Truncated Netpbm pixel data: " + path.string();
    result.image = NetpbmImage{};
    return result;
  }

  result.ok = true;
  return result;
}

[[nodiscard]] inline bool write_pgm(const std::filesystem::path& path, std::uint32_t width, std::uint32_t height,
                                   const std::vector<std::uint8_t>& pixels) {
  if (pixels.size() != static_cast<std::size_t>(width) * height) return false;
  std::ofstream out(path, std::ios::binary);
  if (!out) return false;
  out << "P5\n" << width << ' ' << height << "\n255\n";
  out.write(reinterpret_cast<const char*>(pixels.data()), static_cast<std::streamsize>(pixels.size()));
  return static_cast<bool>(out);
}

[[nodiscard]] inline bool write_ppm(const std::filesystem::path& path, std::uint32_t width, std::uint32_t height,
                                    const std::vector<std::uint8_t>& pixels) {
  if (pixels.size() != static_cast<std::size_t>(width) * height * 3) return false;
  std::ofstream out(path, std::ios::binary);
  if (!out) return false;
  out << "P6\n" << width << ' ' << height << "\n255\n";
  out.write(reinterpret_cast<const char*>(pixels.data()), static_cast<std::streamsize>(pixels.size()));
  return static_cast<bool>(out);
}

}  // namespace cavr::adapters::file_camera
