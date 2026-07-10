#pragma once

// JSON import/export for calibration, using the project's hand-rolled cavr::json
// (the same one MachineProfile and recordings use) so calibrations are portable,
// human-inspectable files that sit next to a machine profile. No third-party deps.

#include <cavr/calibration/camera_intrinsics.hpp>
#include <cavr/calibration/hand_eye.hpp>
#include <cavr/machine/json.hpp>

#include <optional>
#include <string>
#include <string_view>

namespace cavr::calibration {

// -------------------------------------------------------------- intrinsics

[[nodiscard]] inline json::Value to_json(const CameraIntrinsics& in) {
  json::Value out;
  out.set("width", static_cast<std::int64_t>(in.width));
  out.set("height", static_cast<std::int64_t>(in.height));
  out.set("fx", in.fx);
  out.set("fy", in.fy);
  out.set("cx", in.cx);
  out.set("cy", in.cy);
  out.set("distortion", json::Array{in.k1, in.k2, in.p1, in.p2, in.k3});
  return out;
}

[[nodiscard]] inline CameraIntrinsics intrinsics_from_json(const json::Value& v) {
  CameraIntrinsics in;
  in.width = static_cast<std::uint32_t>(v.at("width").as_int(0));
  in.height = static_cast<std::uint32_t>(v.at("height").as_int(0));
  in.fx = v.at("fx").as_number();
  in.fy = v.at("fy").as_number();
  in.cx = v.at("cx").as_number();
  in.cy = v.at("cy").as_number();
  if (const json::Value* d = v.find("distortion"); d && d->is_array()) {
    const auto& a = d->as_array();
    // OpenCV order: [k1, k2, p1, p2, k3].
    if (a.size() > 0) in.k1 = a[0].as_number();
    if (a.size() > 1) in.k2 = a[1].as_number();
    if (a.size() > 2) in.p1 = a[2].as_number();
    if (a.size() > 3) in.p2 = a[3].as_number();
    if (a.size() > 4) in.k3 = a[4].as_number();
  }
  return in;
}

// --------------------------------------------------------------- hand-eye

[[nodiscard]] inline const char* to_string(HandEyeType t) {
  return t == HandEyeType::EyeToHand ? "eye_to_hand" : "eye_in_hand";
}

[[nodiscard]] inline HandEyeType hand_eye_type_from_string(std::string_view s) {
  return s == "eye_to_hand" ? HandEyeType::EyeToHand : HandEyeType::EyeInHand;
}

[[nodiscard]] inline json::Value to_json(const HandEyeCalibration& he) {
  json::Value out;
  out.set("type", to_string(he.type));
  out.set("camera_frame", he.camera_frame);
  out.set("translation", json::Array{he.transform.position_m.x_m, he.transform.position_m.y_m,
                                     he.transform.position_m.z_m});
  out.set("rotation", json::Array{he.transform.orientation.x(), he.transform.orientation.y(),
                                  he.transform.orientation.z(), he.transform.orientation.w()});
  out.set("method", he.method);
  out.set("residual_position_m", he.residual_position_m);
  out.set("residual_rotation_rad", he.residual_rotation_rad);
  out.set("uncertainty_position_m", he.uncertainty_position_m);
  out.set("uncertainty_rotation_rad", he.uncertainty_rotation_rad);
  out.set("version", he.version);
  out.set("calibrated_at_ns", he.calibrated_at_ns);
  return out;
}

[[nodiscard]] inline HandEyeCalibration hand_eye_from_json(const json::Value& v) {
  HandEyeCalibration he;
  he.type = hand_eye_type_from_string(v.at("type").as_string("eye_in_hand"));
  he.camera_frame = v.at("camera_frame").as_string();

  core::Vec3 t{};
  if (const json::Value* tr = v.find("translation"); tr && tr->is_array() && tr->as_array().size() == 3) {
    const auto& a = tr->as_array();
    t = core::Vec3{a[0].as_number(), a[1].as_number(), a[2].as_number()};
  }
  core::Quaternion q = core::Quaternion::identity();
  if (const json::Value* rot = v.find("rotation"); rot && rot->is_array() && rot->as_array().size() == 4) {
    const auto& a = rot->as_array();
    if (auto parsed = core::Quaternion::from_xyzw(a[0].as_number(), a[1].as_number(),
                                                  a[2].as_number(), a[3].as_number(), 1.0e-3)) {
      q = *parsed;
    }
  }
  he.transform = core::Pose3D{t, q};

  he.method = v.at("method").as_string();
  he.residual_position_m = v.at("residual_position_m").as_number();
  he.residual_rotation_rad = v.at("residual_rotation_rad").as_number();
  he.uncertainty_position_m = v.at("uncertainty_position_m").as_number();
  he.uncertainty_rotation_rad = v.at("uncertainty_rotation_rad").as_number();
  he.version = static_cast<int>(v.at("version").as_int(1));
  he.calibrated_at_ns = v.at("calibrated_at_ns").as_int(0);
  return he;
}

// ----------------------------------------------------- string convenience

[[nodiscard]] inline std::string intrinsics_to_string(const CameraIntrinsics& in, int indent = 2) {
  return to_json(in).dump(indent);
}

[[nodiscard]] inline std::optional<CameraIntrinsics> intrinsics_from_string(std::string_view text) {
  std::string error;
  auto parsed = json::parse(text, error);
  if (!parsed) return std::nullopt;
  return intrinsics_from_json(*parsed);
}

[[nodiscard]] inline std::string hand_eye_to_string(const HandEyeCalibration& he, int indent = 2) {
  return to_json(he).dump(indent);
}

[[nodiscard]] inline std::optional<HandEyeCalibration> hand_eye_from_string(std::string_view text) {
  std::string error;
  auto parsed = json::parse(text, error);
  if (!parsed) return std::nullopt;
  return hand_eye_from_json(*parsed);
}

}  // namespace cavr::calibration
