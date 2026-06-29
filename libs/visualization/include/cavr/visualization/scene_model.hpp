#pragma once

#include <array>
#include <string_view>

namespace cavr::visualization {

struct Vec3 {
  double x{};
  double y{};
  double z{};
};

struct ColorRgb {
  double r{};
  double g{};
  double b{};
};

struct FrameEntry {
  std::string_view name;
  ColorRgb color;
};

[[nodiscard]] constexpr auto default_frame_tree() {
  return std::array{
      FrameEntry{"world", ColorRgb{0.35, 0.52, 1.0}},
      FrameEntry{"base", ColorRgb{0.55, 0.61, 0.68}},
      FrameEntry{"flange", ColorRgb{0.82, 0.72, 0.64}},
      FrameEntry{"tcp", ColorRgb{1.0, 0.30, 0.27}},
      FrameEntry{"camera", ColorRgb{0.33, 0.84, 0.42}},
      FrameEntry{"object", ColorRgb{0.30, 0.79, 0.94}},
  };
}

}  // namespace cavr::visualization
