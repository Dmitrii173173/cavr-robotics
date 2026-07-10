#pragma once

// Point clouds on the recording: serialize a PointCloud to/from the project's JSON
// and read the point-cloud channel back out of a recording. Clouds travel as their
// own message stream alongside robot/telemetry and camera color, time-stamped on
// the same clock, so a recording carries synchronized robot + vision + 3D geometry.
//
// Payloads here are JSON (uniform across the JSON and MCAP backends and easy to
// inspect); a production scanner would instead store a packed binary point buffer
// as the message payload with the layout named on the channel schema.

#include <cavr/adapter_sdk/point_cloud.hpp>
#include <cavr/machine/json.hpp>
#include <cavr/record/reader.hpp>

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace cavr::runtime {

namespace sdk = cavr::adapter_sdk;

namespace detail {

inline constexpr std::string_view kPointCloudSchema = "cavr.adapter_sdk.PointCloud";

[[nodiscard]] inline json::Value point_cloud_to_json(const sdk::PointCloud& cloud) {
  json::Value j;
  j.set("t_ns", static_cast<std::int64_t>(cloud.timestamp.nanoseconds()));
  j.set("frame_id", cloud.frame_id);

  // Points as a flat [x0,y0,z0, x1,y1,z1, ...] array — compact and unambiguous.
  json::Array points;
  points.reserve(cloud.points.size() * 3);
  for (const auto& p : cloud.points) {
    points.push_back(p.x_m);
    points.push_back(p.y_m);
    points.push_back(p.z_m);
  }
  j.set("points", std::move(points));

  if (cloud.has_colors()) {
    json::Array colors;
    colors.reserve(cloud.colors.size() * 3);
    for (const auto& c : cloud.colors) {
      colors.push_back(static_cast<std::int64_t>(c.r));
      colors.push_back(static_cast<std::int64_t>(c.g));
      colors.push_back(static_cast<std::int64_t>(c.b));
    }
    j.set("colors", std::move(colors));
  }
  if (cloud.has_normals()) {
    json::Array normals;
    normals.reserve(cloud.normals.size() * 3);
    for (const auto& n : cloud.normals) {
      normals.push_back(n.x_m);
      normals.push_back(n.y_m);
      normals.push_back(n.z_m);
    }
    j.set("normals", std::move(normals));
  }
  return j;
}

[[nodiscard]] inline sdk::PointCloud point_cloud_from_json(const json::Value& j) {
  sdk::PointCloud cloud;
  cloud.timestamp = core::Timestamp::from_nanoseconds(j.at("t_ns").as_int());
  cloud.frame_id = j.at("frame_id").as_string();

  if (const json::Value* points = j.find("points"); points && points->is_array()) {
    const auto& a = points->as_array();
    for (std::size_t i = 0; i + 2 < a.size(); i += 3) {
      cloud.points.push_back(core::Vec3{a[i].as_number(), a[i + 1].as_number(), a[i + 2].as_number()});
    }
  }
  if (const json::Value* colors = j.find("colors"); colors && colors->is_array()) {
    const auto& a = colors->as_array();
    for (std::size_t i = 0; i + 2 < a.size(); i += 3) {
      cloud.colors.push_back(sdk::PointColor{static_cast<std::uint8_t>(a[i].as_int()),
                                             static_cast<std::uint8_t>(a[i + 1].as_int()),
                                             static_cast<std::uint8_t>(a[i + 2].as_int())});
    }
  }
  if (const json::Value* normals = j.find("normals"); normals && normals->is_array()) {
    const auto& a = normals->as_array();
    for (std::size_t i = 0; i + 2 < a.size(); i += 3) {
      cloud.normals.push_back(core::Vec3{a[i].as_number(), a[i + 1].as_number(), a[i + 2].as_number()});
    }
  }
  return cloud;
}

}  // namespace detail

// Reads the point-cloud channel out of a recording, ordered by log_time.
[[nodiscard]] inline std::vector<sdk::PointCloud> read_point_clouds(const record::RecordingReader& reader) {
  std::vector<sdk::PointCloud> clouds;
  if (const record::Channel* channel = reader.find_channel(record::topics::kCameraPoints)) {
    for (const auto& m : reader.messages_on(channel->id)) {
      std::string error;
      auto value = json::parse(m.data, error);
      if (value) clouds.push_back(detail::point_cloud_from_json(*value));
    }
  }
  return clouds;
}

}  // namespace cavr::runtime
