#pragma once

// Camera frames on the recording: serialize a CameraFrame to/from the project's
// JSON and read the camera/color channel back out of a recording. Frames travel
// as their own message stream alongside robot/telemetry, time-stamped on the same
// clock, so a recording carries a synchronized robot + vision session.
//
// Payloads here are JSON (uniform across the JSON and MCAP backends and easy to
// inspect); a production high-resolution camera would instead store raw image
// bytes as the message payload with the format named on the channel schema.

#include <cavr/adapter_sdk/camera_frame.hpp>
#include <cavr/machine/json.hpp>
#include <cavr/record/reader.hpp>

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace cavr::runtime {

namespace sdk = cavr::adapter_sdk;

namespace detail {

inline constexpr std::string_view kCameraSchema = "cavr.adapter_sdk.CameraFrame";

[[nodiscard]] inline json::Value camera_frame_to_json(const sdk::CameraFrame& frame) {
  json::Value j;
  j.set("t_ns", static_cast<std::int64_t>(frame.timestamp.nanoseconds()));
  j.set("frame_id", frame.frame_id);
  j.set("width", static_cast<std::int64_t>(frame.width));
  j.set("height", static_cast<std::int64_t>(frame.height));
  j.set("encoding", frame.encoding);
  json::Array pixels;
  for (std::uint8_t b : frame.pixels) pixels.push_back(static_cast<std::int64_t>(b));
  j.set("pixels", std::move(pixels));
  return j;
}

[[nodiscard]] inline sdk::CameraFrame camera_frame_from_json(const json::Value& j) {
  sdk::CameraFrame frame;
  frame.timestamp = core::Timestamp::from_nanoseconds(j.at("t_ns").as_int());
  frame.frame_id = j.at("frame_id").as_string();
  frame.width = static_cast<std::uint32_t>(j.at("width").as_int());
  frame.height = static_cast<std::uint32_t>(j.at("height").as_int());
  frame.encoding = j.at("encoding").as_string();
  if (const json::Value* pixels = j.find("pixels"); pixels && pixels->is_array()) {
    for (const auto& v : pixels->as_array()) {
      frame.pixels.push_back(static_cast<std::uint8_t>(v.as_int()));
    }
  }
  return frame;
}

}  // namespace detail

// Reads the camera/color channel out of a recording, ordered by log_time.
[[nodiscard]] inline std::vector<sdk::CameraFrame> read_camera_frames(
    const record::RecordingReader& reader) {
  std::vector<sdk::CameraFrame> frames;
  if (const record::Channel* camera = reader.find_channel(record::topics::kCameraColor)) {
    for (const auto& m : reader.messages_on(camera->id)) {
      std::string error;
      auto value = json::parse(m.data, error);
      if (value) frames.push_back(detail::camera_frame_from_json(*value));
    }
  }
  return frames;
}

}  // namespace cavr::runtime
