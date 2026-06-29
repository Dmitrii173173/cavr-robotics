#pragma once

// Import/export for MachineProfile. Profiles are stored as JSON so they are
// human-readable, diff-friendly and controller-neutral. Loading collects errors
// in the same style as cavr::replay::LoadResult.

#include <cavr/machine/json.hpp>
#include <cavr/machine/machine_profile.hpp>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace cavr::machine {

namespace detail {

[[nodiscard]] inline json::Value vec3_to_json(const core::Vec3& v) {
  return json::Array{v.x_m, v.y_m, v.z_m};
}

[[nodiscard]] inline core::Vec3 vec3_from_json(const json::Value& v) {
  if (v.is_array() && v.as_array().size() == 3) {
    const auto& a = v.as_array();
    return core::Vec3{a[0].as_number(), a[1].as_number(), a[2].as_number()};
  }
  return core::Vec3{};
}

[[nodiscard]] inline json::Value pose_to_json(const core::Pose3D& p) {
  json::Value out;
  out.set("position", vec3_to_json(p.position_m));
  out.set("orientation", json::Array{p.orientation.x(), p.orientation.y(),
                                     p.orientation.z(), p.orientation.w()});
  return out;
}

[[nodiscard]] inline core::Pose3D pose_from_json(const json::Value& v) {
  core::Pose3D pose;
  pose.position_m = vec3_from_json(v.at("position"));
  const json::Value& q = v.at("orientation");
  if (q.is_array() && q.as_array().size() == 4) {
    const auto& a = q.as_array();
    if (auto parsed = core::Quaternion::from_xyzw(a[0].as_number(), a[1].as_number(),
                                                  a[2].as_number(), a[3].as_number(), 1.0e-3)) {
      pose.orientation = *parsed;
    }
  }
  return pose;
}

}  // namespace detail

[[nodiscard]] inline json::Value to_json(const MachineProfile& p) {
  using detail::pose_to_json;
  using detail::vec3_to_json;

  json::Value root;
  root.set("schema_version", p.schema_version);
  root.set("id", p.id);
  root.set("display_name", p.display_name);
  root.set("robot_model", p.robot_model);
  root.set("controller", p.controller);
  root.set("asset", p.asset);

  json::Array axes;
  for (const auto& a : p.axes) {
    json::Value j;
    j.set("name", a.name);
    j.set("type", to_string(a.type));
    j.set("axis", vec3_to_json(a.axis));
    j.set("origin_m", vec3_to_json(a.origin_m));
    j.set("lower_limit", a.lower_limit);
    j.set("upper_limit", a.upper_limit);
    j.set("max_speed", a.max_speed);
    j.set("controller_variable", a.controller_variable);
    axes.push_back(std::move(j));
  }
  root.set("axes", std::move(axes));

  json::Array frames;
  for (const auto& f : p.frames) {
    json::Value j;
    j.set("name", f.name);
    j.set("kind", to_string(f.kind));
    j.set("parent", f.parent);
    j.set("transform", pose_to_json(f.transform));
    frames.push_back(std::move(j));
  }
  root.set("frames", std::move(frames));

  json::Array io;
  for (const auto& c : p.io) {
    json::Value j;
    j.set("name", c.name);
    j.set("kind", to_string(c.kind));
    j.set("direction", to_string(c.direction));
    j.set("index", c.index);
    j.set("controller_variable", c.controller_variable);
    io.push_back(std::move(j));
  }
  root.set("io", std::move(io));

  json::Array telemetry;
  for (const auto& c : p.telemetry) {
    json::Value j;
    j.set("name", c.name);
    j.set("kind", to_string(c.kind));
    j.set("unit", c.unit);
    j.set("rate_hz", c.rate_hz);
    j.set("controller_variable", c.controller_variable);
    telemetry.push_back(std::move(j));
  }
  root.set("telemetry", std::move(telemetry));

  json::Array cameras;
  for (const auto& c : p.cameras) {
    json::Value j;
    j.set("name", c.name);
    j.set("mounted_frame", c.mounted_frame);
    j.set("provides_depth", c.provides_depth);
    j.set("provides_point_cloud", c.provides_point_cloud);
    j.set("hand_eye_calibrated", c.hand_eye_calibrated);
    j.set("extrinsics", pose_to_json(c.extrinsics));
    j.set("intrinsics_ref", c.intrinsics_ref);
    cameras.push_back(std::move(j));
  }
  root.set("cameras", std::move(cameras));

  json::Array motion;
  for (const auto& m : p.motion) {
    json::Value j;
    j.set("kind", to_string(m.kind));
    j.set("supported", m.supported);
    j.set("default_speed", m.default_speed);
    motion.push_back(std::move(j));
  }
  root.set("motion", std::move(motion));

  json::Value weld;
  weld.set("enabled", p.weld.enabled);
  weld.set("travel_speed_mm_s", p.weld.travel_speed_mm_s);
  weld.set("segment_length_mm", p.weld.segment_length_mm);
  weld.set("settle_delay_s", p.weld.settle_delay_s);
  weld.set("tolerance_mm", p.weld.tolerance_mm);
  weld.set("process_program", p.weld.process_program);
  root.set("weld", std::move(weld));

  return root;
}

[[nodiscard]] inline MachineProfile profile_from_json(const json::Value& root) {
  using detail::pose_from_json;
  using detail::vec3_from_json;

  MachineProfile p;
  p.schema_version = static_cast<int>(root.at("schema_version").as_int(1));
  p.id = root.at("id").as_string();
  p.display_name = root.at("display_name").as_string();
  p.robot_model = root.at("robot_model").as_string();
  p.controller = root.at("controller").as_string();
  p.asset = root.at("asset").as_string();

  if (const json::Value* axes = root.find("axes"); axes && axes->is_array()) {
    for (const auto& j : axes->as_array()) {
      AxisSpec a;
      a.name = j.at("name").as_string();
      a.type = joint_type_from_string(j.at("type").as_string("revolute"));
      a.axis = vec3_from_json(j.at("axis"));
      a.origin_m = vec3_from_json(j.at("origin_m"));
      a.lower_limit = j.at("lower_limit").as_number(a.lower_limit);
      a.upper_limit = j.at("upper_limit").as_number(a.upper_limit);
      a.max_speed = j.at("max_speed").as_number(a.max_speed);
      a.controller_variable = j.at("controller_variable").as_string();
      p.axes.push_back(std::move(a));
    }
  }
  if (const json::Value* frames = root.find("frames"); frames && frames->is_array()) {
    for (const auto& j : frames->as_array()) {
      CoordinateFrame f;
      f.name = j.at("name").as_string();
      f.kind = frame_kind_from_string(j.at("kind").as_string("user"));
      f.parent = j.at("parent").as_string();
      f.transform = pose_from_json(j.at("transform"));
      p.frames.push_back(std::move(f));
    }
  }
  if (const json::Value* io = root.find("io"); io && io->is_array()) {
    for (const auto& j : io->as_array()) {
      IoChannel c;
      c.name = j.at("name").as_string();
      c.kind = io_kind_from_string(j.at("kind").as_string("digital"));
      c.direction = io_direction_from_string(j.at("direction").as_string("input"));
      c.index = static_cast<int>(j.at("index").as_int());
      c.controller_variable = j.at("controller_variable").as_string();
      p.io.push_back(std::move(c));
    }
  }
  if (const json::Value* tele = root.find("telemetry"); tele && tele->is_array()) {
    for (const auto& j : tele->as_array()) {
      TelemetryChannel c;
      c.name = j.at("name").as_string();
      c.kind = channel_kind_from_string(j.at("kind").as_string("custom"));
      c.unit = j.at("unit").as_string();
      c.rate_hz = j.at("rate_hz").as_number();
      c.controller_variable = j.at("controller_variable").as_string();
      p.telemetry.push_back(std::move(c));
    }
  }
  if (const json::Value* cams = root.find("cameras"); cams && cams->is_array()) {
    for (const auto& j : cams->as_array()) {
      CameraConfig c;
      c.name = j.at("name").as_string();
      c.mounted_frame = j.at("mounted_frame").as_string();
      c.provides_depth = j.at("provides_depth").as_bool();
      c.provides_point_cloud = j.at("provides_point_cloud").as_bool();
      c.hand_eye_calibrated = j.at("hand_eye_calibrated").as_bool();
      c.extrinsics = pose_from_json(j.at("extrinsics"));
      c.intrinsics_ref = j.at("intrinsics_ref").as_string();
      p.cameras.push_back(std::move(c));
    }
  }
  if (const json::Value* motion = root.find("motion"); motion && motion->is_array()) {
    for (const auto& j : motion->as_array()) {
      MotionCapability m;
      m.kind = motion_kind_from_string(j.at("kind").as_string("movej"));
      m.supported = j.at("supported").as_bool(true);
      m.default_speed = j.at("default_speed").as_number();
      p.motion.push_back(std::move(m));
    }
  }
  if (const json::Value* weld = root.find("weld"); weld && weld->is_object()) {
    p.weld.enabled = weld->at("enabled").as_bool();
    p.weld.travel_speed_mm_s = weld->at("travel_speed_mm_s").as_number(p.weld.travel_speed_mm_s);
    p.weld.segment_length_mm = weld->at("segment_length_mm").as_number(p.weld.segment_length_mm);
    p.weld.settle_delay_s = weld->at("settle_delay_s").as_number(p.weld.settle_delay_s);
    p.weld.tolerance_mm = weld->at("tolerance_mm").as_number(p.weld.tolerance_mm);
    p.weld.process_program = weld->at("process_program").as_string();
  }
  return p;
}

struct ProfileLoadResult final {
  MachineProfile profile;
  std::vector<std::string> errors;
  [[nodiscard]] bool ok() const noexcept { return errors.empty(); }
};

[[nodiscard]] inline std::string export_profile_string(const MachineProfile& p) {
  return to_json(p).dump(2);
}

[[nodiscard]] inline bool save_profile(const MachineProfile& p, const std::filesystem::path& path,
                                       std::vector<std::string>& errors) {
  std::ofstream out(path);
  if (!out) {
    errors.push_back("Failed to open profile for writing: " + path.string());
    return false;
  }
  out << export_profile_string(p) << '\n';
  return true;
}

[[nodiscard]] inline ProfileLoadResult load_profile(const std::filesystem::path& path) {
  ProfileLoadResult result;
  std::ifstream in(path);
  if (!in) {
    result.errors.push_back("Failed to open profile: " + path.string());
    return result;
  }
  std::ostringstream buffer;
  buffer << in.rdbuf();
  std::string parse_error;
  auto value = json::parse(buffer.str(), parse_error);
  if (!value) {
    result.errors.push_back("Invalid profile JSON: " + parse_error);
    return result;
  }
  result.profile = profile_from_json(*value);
  if (result.profile.axes.empty()) {
    result.errors.push_back("Profile has no axes");
  }
  return result;
}

[[nodiscard]] inline ProfileLoadResult parse_profile(std::string_view text) {
  ProfileLoadResult result;
  std::string parse_error;
  auto value = json::parse(text, parse_error);
  if (!value) {
    result.errors.push_back("Invalid profile JSON: " + parse_error);
    return result;
  }
  result.profile = profile_from_json(*value);
  return result;
}

}  // namespace cavr::machine
