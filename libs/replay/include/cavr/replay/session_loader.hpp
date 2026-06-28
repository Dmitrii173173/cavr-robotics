#pragma once

#include <cavr/replay/session.hpp>

#include <algorithm>
#include <charconv>
#include <fstream>
#include <regex>
#include <sstream>
#include <string>
#include <string_view>

namespace cavr::replay {
namespace detail {

[[nodiscard]] inline std::string trim(std::string_view value) {
  const auto begin = value.find_first_not_of(" \t\r\n");
  if (begin == std::string_view::npos) {
    return {};
  }
  const auto end = value.find_last_not_of(" \t\r\n");
  return std::string(value.substr(begin, end - begin + 1));
}

[[nodiscard]] inline std::vector<std::string> split_csv_line(std::string_view line) {
  std::vector<std::string> fields;
  std::size_t start = 0;
  while (start <= line.size()) {
    const std::size_t comma = line.find(',', start);
    const std::size_t end = comma == std::string_view::npos ? line.size() : comma;
    fields.push_back(trim(line.substr(start, end - start)));
    if (comma == std::string_view::npos) {
      break;
    }
    start = comma + 1;
  }
  return fields;
}

[[nodiscard]] inline bool parse_i64(std::string_view text, std::int64_t& output) noexcept {
  const auto* begin = text.data();
  const auto* end = text.data() + text.size();
  const auto result = std::from_chars(begin, end, output);
  return result.ec == std::errc{} && result.ptr == end;
}

[[nodiscard]] inline bool parse_double(std::string_view text, double& output) noexcept {
  std::string owned(text);
  char* end = nullptr;
  output = std::strtod(owned.c_str(), &end);
  return end == owned.c_str() + owned.size() && std::isfinite(output);
}

[[nodiscard]] inline std::string read_text_file(const std::filesystem::path& path, std::vector<std::string>& errors) {
  std::ifstream input(path);
  if (!input) {
    errors.push_back("Failed to open file: " + path.string());
    return {};
  }
  std::ostringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
}

[[nodiscard]] inline std::optional<std::string> extract_string_field(
    const std::string& text, const std::string& key, std::vector<std::string>& errors) {
  const std::regex expression("\\\"" + key + "\\\"\\s*:\\s*\\\"([^\\\"]+)\\\"");
  std::smatch match;
  if (!std::regex_search(text, match, expression)) {
    errors.push_back("Missing string field in manifest: " + key);
    return std::nullopt;
  }
  return match[1].str();
}

[[nodiscard]] inline std::vector<ImageSample> extract_images(
    const std::string& text, const std::filesystem::path& image_directory, std::vector<std::string>& errors) {
  std::vector<ImageSample> images;
  const std::regex image_expression(
      R"json(\{[^\}]*"timestamp_ns"\s*:\s*(-?[0-9]+)[^\}]*"path"\s*:\s*"([^"]+)"[^\}]*\})json");

  for (std::sregex_iterator it(text.begin(), text.end(), image_expression), end; it != end; ++it) {
    std::int64_t timestamp_ns{};
    const std::string timestamp_text = (*it)[1].str();
    if (!parse_i64(timestamp_text, timestamp_ns)) {
      errors.push_back("Invalid image timestamp: " + timestamp_text);
      continue;
    }
    images.push_back(ImageSample{
        core::Timestamp::from_nanoseconds(timestamp_ns),
        image_directory / (*it)[2].str(),
    });
  }

  if (images.empty()) {
    errors.push_back("Manifest does not contain any image samples");
  }
  return images;
}

[[nodiscard]] inline std::vector<PoseSample> load_pose_csv(
    const std::filesystem::path& csv_path, std::vector<std::string>& errors) {
  std::ifstream input(csv_path);
  if (!input) {
    errors.push_back("Failed to open pose CSV: " + csv_path.string());
    return {};
  }

  std::vector<PoseSample> poses;
  std::string line;
  std::size_t line_number = 0;
  while (std::getline(input, line)) {
    ++line_number;
    if (line.empty()) {
      continue;
    }
    if (line_number == 1 && line.find("timestamp_ns") != std::string::npos) {
      continue;
    }

    const auto fields = split_csv_line(line);
    if (fields.size() != 8) {
      errors.push_back("Pose CSV line " + std::to_string(line_number) + " must contain 8 fields");
      continue;
    }

    std::int64_t timestamp_ns{};
    double x_m{};
    double y_m{};
    double z_m{};
    double qx{};
    double qy{};
    double qz{};
    double qw{};
    if (!parse_i64(fields[0], timestamp_ns) || !parse_double(fields[1], x_m) ||
        !parse_double(fields[2], y_m) || !parse_double(fields[3], z_m) ||
        !parse_double(fields[4], qx) || !parse_double(fields[5], qy) ||
        !parse_double(fields[6], qz) || !parse_double(fields[7], qw)) {
      errors.push_back("Pose CSV line " + std::to_string(line_number) + " contains invalid numeric data");
      continue;
    }

    auto orientation = core::Quaternion::from_xyzw(qx, qy, qz, qw, 1.0e-6);
    if (!orientation.has_value()) {
      errors.push_back("Pose CSV line " + std::to_string(line_number) + " contains a non-normalized quaternion");
      continue;
    }

    poses.push_back(PoseSample{
        core::Timestamp::from_nanoseconds(timestamp_ns),
        core::Pose3D{core::Vec3{x_m, y_m, z_m}, *orientation},
    });
  }

  if (poses.empty()) {
    errors.push_back("Pose CSV does not contain any pose samples");
  }
  return poses;
}

[[nodiscard]] inline std::vector<ReplaySample> make_replay_samples(
    const std::vector<PoseSample>& poses, const std::vector<ImageSample>& images) {
  std::vector<ReplaySample> samples;
  std::size_t pose_index = 0;
  std::size_t image_index = 0;
  while (pose_index < poses.size() && image_index < images.size()) {
    const auto pose_time = poses[pose_index].timestamp.nanoseconds();
    const auto image_time = images[image_index].timestamp.nanoseconds();
    if (pose_time == image_time) {
      samples.push_back(ReplaySample{poses[pose_index].timestamp, pose_index, poses[pose_index].pose, images[image_index].path});
      ++pose_index;
      ++image_index;
    } else if (pose_time < image_time) {
      ++pose_index;
    } else {
      ++image_index;
    }
  }
  return samples;
}

}  // namespace detail

[[nodiscard]] inline LoadResult load_session_manifest(const std::filesystem::path& manifest_path) {
  LoadResult result;
  result.session.manifest.manifest_path = manifest_path;

  const std::string manifest_text = detail::read_text_file(manifest_path, result.errors);
  if (!result.errors.empty()) {
    return result;
  }

  const auto poses_csv = detail::extract_string_field(manifest_text, "poses_csv", result.errors);
  const auto images_dir = detail::extract_string_field(manifest_text, "images_dir", result.errors);
  if (!poses_csv.has_value() || !images_dir.has_value()) {
    return result;
  }

  const auto base_dir = manifest_path.parent_path();
  result.session.manifest.poses_csv_path = base_dir / *poses_csv;
  result.session.manifest.images_directory = base_dir / *images_dir;
  result.session.manifest.poses = detail::load_pose_csv(result.session.manifest.poses_csv_path, result.errors);
  result.session.manifest.images = detail::extract_images(manifest_text, result.session.manifest.images_directory, result.errors);

  std::sort(result.session.manifest.poses.begin(), result.session.manifest.poses.end(),
            [](const PoseSample& lhs, const PoseSample& rhs) { return lhs.timestamp < rhs.timestamp; });
  std::sort(result.session.manifest.images.begin(), result.session.manifest.images.end(),
            [](const ImageSample& lhs, const ImageSample& rhs) { return lhs.timestamp < rhs.timestamp; });

  if (!result.errors.empty()) {
    return result;
  }

  result.session.replay_samples = detail::make_replay_samples(result.session.manifest.poses, result.session.manifest.images);
  if (result.session.replay_samples.empty()) {
    result.errors.push_back("Session does not contain synchronized pose and image samples");
  }

  return result;
}

}  // namespace cavr::replay
