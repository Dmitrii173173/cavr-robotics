#include "VisualProgramModel.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <utility>

namespace {

constexpr double kPi = 3.14159265358979323846;

QVariantMap base_map(const QString& severity, const QString& code, const QString& message,
                     int feature_index, const cavr::core::Vec3& position) {
  QVariantMap m;
  m["severity"] = severity;
  m["code"] = code;
  m["message"] = message;
  m["featureIndex"] = feature_index;
  m["x"] = position.x_m;
  m["y"] = position.y_m;
  m["z"] = position.z_m;
  return m;
}

bool inside_box(const cavr::core::Vec3& p, const cavr::core::Vec3& center,
                const cavr::core::Vec3& size) {
  return std::abs(p.x_m - center.x_m) <= size.x_m * 0.5 &&
         std::abs(p.y_m - center.y_m) <= size.y_m * 0.5 &&
         std::abs(p.z_m - center.z_m) <= size.z_m * 0.5;
}

}  // namespace

VisualProgramModel::VisualProgramModel() {
  load_demo_workpiece();
}

void VisualProgramModel::load_demo_workpiece() {
  features_.clear();
  zones_.clear();
  point_cloud_.clear();
  operations_.clear();
  diagnostics_.clear();
  selected_feature_ = 0;
  selected_operation_ = -1;

  features_.push_back({0, "seam_top_long", "Top lap seam A", "line",
                       {0.46, -0.18, 0.44}, {0.82, -0.18, 0.44},
                       {0.64, -0.18, 0.44}, {0.0, 0.0, 1.0}, 0.018, true});
  features_.push_back({1, "seam_inner_corner", "Inside corner seam B", "edge",
                       {0.50, 0.04, 0.46}, {0.80, 0.04, 0.46},
                       {0.65, 0.04, 0.46}, {0.0, -0.4, 0.9}, 0.016, true});
  features_.push_back({2, "vertical_return", "Return edge seam C", "line",
                       {0.82, -0.18, 0.44}, {0.82, 0.04, 0.46},
                       {0.82, -0.07, 0.45}, {0.0, 0.0, 1.0}, 0.016, true});
  features_.push_back({3, "top_face_scan", "Top face contour scan", "face",
                       {0.48, -0.06, 0.49}, {0.78, -0.06, 0.49},
                       {0.63, -0.06, 0.49}, {0.0, 0.0, 1.0}, 0.12, true});

  zones_.push_back({0, "work", "Robot work envelope", "work",
                    {0.65, -0.02, 0.52}, {0.90, 0.72, 0.92}, "#2f8cff"});
  zones_.push_back({1, "slow", "Slowdown near fixture", "slow",
                    {0.64, -0.20, 0.43}, {0.50, 0.18, 0.22}, "#ffb020"});
  zones_.push_back({2, "weld", "Qualified weld window", "weld",
                    {0.65, -0.08, 0.47}, {0.48, 0.34, 0.18}, "#29c782"});
  zones_.push_back({3, "forbidden", "Forbidden clamp volume", "forbidden",
                    {0.88, 0.07, 0.43}, {0.16, 0.18, 0.26}, "#ff4d5e"});
  zones_.push_back({4, "safety", "Operator safety boundary", "safety",
                    {0.50, -0.45, 0.50}, {0.34, 0.16, 0.70}, "#b45cff"});

  for (int ix = 0; ix < 10; ++ix) {
    for (int iy = 0; iy < 6; ++iy) {
      const double jitter = ((ix + iy) % 3 - 1) * 0.003;
      point_cloud_.push_back({0.45 + ix * 0.043, -0.22 + iy * 0.052,
                              0.50 + jitter});
    }
  }
  validate_current();
}

void VisualProgramModel::clear_operations() {
  operations_.clear();
  selected_operation_ = -1;
  validate_current();
}

bool VisualProgramModel::select_feature(int index) {
  if (!feature_by_index(index)) return false;
  selected_feature_ = index;
  selected_operation_ = -1;
  validate_current();
  return true;
}

bool VisualProgramModel::select_operation(int index) {
  if (index < 0 || index >= static_cast<int>(operations_.size())) return false;
  selected_operation_ = index;
  selected_feature_ = operations_[static_cast<std::size_t>(index)].feature_index;
  params_ = operations_[static_cast<std::size_t>(index)].params;
  validate_current();
  return true;
}

void VisualProgramModel::set_operation_params(const QString& type, double speed_mm_s,
                                              double torch_angle_deg, double standoff_mm,
                                              double approach_mm, double retract_mm,
                                              const QString& direction,
                                              const QString& weld_mode) {
  params_.type = type;
  params_.speed_mm_s = std::clamp(speed_mm_s, 1.0, 2500.0);
  params_.torch_angle_deg = std::clamp(torch_angle_deg, -80.0, 80.0);
  params_.standoff_mm = std::clamp(standoff_mm, 0.0, 200.0);
  params_.approach_mm = std::clamp(approach_mm, 0.0, 500.0);
  params_.retract_mm = std::clamp(retract_mm, 0.0, 500.0);
  params_.direction = direction;
  params_.weld_mode = weld_mode;

  if (selected_operation_ >= 0 &&
      selected_operation_ < static_cast<int>(operations_.size())) {
    auto& op = operations_[static_cast<std::size_t>(selected_operation_)];
    op.params = params_;
    op.estimated_s = distance(op.start, op.end) / (params_.speed_mm_s / 1000.0);
  }
  validate_current();
}

QVariantMap VisualProgramModel::operation_params() const {
  QVariantMap m;
  m["type"] = params_.type;
  m["speed"] = params_.speed_mm_s;
  m["torchAngle"] = params_.torch_angle_deg;
  m["standoff"] = params_.standoff_mm;
  m["approach"] = params_.approach_mm;
  m["retract"] = params_.retract_mm;
  m["direction"] = params_.direction;
  m["weldMode"] = params_.weld_mode;
  m["selectedFeature"] = selected_feature_;
  m["selectedOperation"] = selected_operation_;
  if (const Feature* f = selected_feature_ptr()) {
    m["featureName"] = f->name;
    m["featureKind"] = f->kind;
  }
  return m;
}

QVariantList VisualProgramModel::features() const {
  QVariantList out;
  for (const auto& f : features_) out.push_back(feature_map(f));
  return out;
}

QVariantList VisualProgramModel::zones() const {
  QVariantList out;
  for (const auto& z : zones_) out.push_back(zone_map(z));
  return out;
}

QVariantList VisualProgramModel::point_cloud() const {
  QVariantList out;
  int i = 0;
  for (const auto& p : point_cloud_) {
    QVariantMap m = vec_map(p);
    m["index"] = i++;
    out.push_back(m);
  }
  return out;
}

QVariantList VisualProgramModel::operations() const {
  QVariantList out;
  for (const auto& op : operations_) out.push_back(operation_map(op));
  return out;
}

QVariantList VisualProgramModel::path_segments() const {
  QVariantList out;
  for (const auto& op : operations_) {
    const bool selected = op.index == selected_operation_;
    const double a = op.params.approach_mm / 1000.0;
    const double r = op.params.retract_mm / 1000.0;
    const cavr::core::Vec3 approach{op.start.x_m, op.start.y_m, op.start.z_m + a};
    const cavr::core::Vec3 retract{op.end.x_m, op.end.y_m, op.end.z_m + r};
    out.push_back(segment_map("move", "move", approach, op.start, "#2f8cff", selected));
    out.push_back(segment_map("approach", "approach", approach, op.start, "#64c7ff", selected));
    out.push_back(segment_map(op.params.type, "weld", op.start, op.end,
                              op.params.type.contains("Weld") ? "#29d47d" : "#e6ecf2",
                              selected));
    out.push_back(segment_map("retract", "retract", op.end, retract, "#ffb020", selected));
  }
  return out;
}

QVariantList VisualProgramModel::diagnostics() const {
  QVariantList out;
  for (const auto& d : diagnostics_) out.push_back(diagnostic_map(d));
  return out;
}

cavr::machine::MotionTask VisualProgramModel::compiled_task() const {
  cavr::machine::MotionTask task;
  for (const auto& op : operations_) {
    const Feature* f = feature_by_index(op.feature_index);
    if (!f) continue;
    auto commands = build_commands(*f, op.params);
    task.insert(task.end(), std::make_move_iterator(commands.begin()),
                std::make_move_iterator(commands.end()));
  }
  return task;
}

VisualProgramModel::BuildResult VisualProgramModel::create_operation_from_selection() {
  const Feature* feature = selected_feature_ptr();
  if (!feature) {
    return {false, "Select a line, edge, face, or point-cloud segment first", {}};
  }
  if (feature->kind == "face" && params_.type.contains("Weld")) {
    diagnostics_.push_back({"warning", "GEOMETRY_FACE_WELD",
                            "Selected face will be converted to a center contour. Pick an edge for production welding.",
                            feature->index, feature->center});
  }

  Operation op;
  op.index = static_cast<int>(operations_.size());
  op.feature_index = feature->index;
  op.stage = operations_.empty() ? "Preparation / first weld" : "Next operation";
  op.name = params_.type + " on " + feature->name;
  op.params = params_;
  op.start = params_.direction == "Reverse" ? feature->end : feature->start;
  op.end = params_.direction == "Reverse" ? feature->start : feature->end;
  op.estimated_s = std::max(0.1, distance(op.start, op.end) / (params_.speed_mm_s / 1000.0));
  operations_.push_back(op);
  selected_operation_ = op.index;
  selected_feature_ = feature->index;

  validate_current();
  BuildResult result;
  result.ok = true;
  result.message = op.name;
  result.commands = compiled_task();
  return result;
}

const VisualProgramModel::Feature* VisualProgramModel::selected_feature_ptr() const {
  return feature_by_index(selected_feature_);
}

const VisualProgramModel::Feature* VisualProgramModel::feature_by_index(int index) const {
  for (const auto& f : features_) {
    if (f.index == index) return &f;
  }
  return nullptr;
}

double VisualProgramModel::distance(const cavr::core::Vec3& a, const cavr::core::Vec3& b) {
  const double dx = a.x_m - b.x_m;
  const double dy = a.y_m - b.y_m;
  const double dz = a.z_m - b.z_m;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

cavr::core::Vec3 VisualProgramModel::lerp(const cavr::core::Vec3& a,
                                          const cavr::core::Vec3& b, double t) {
  return {a.x_m + (b.x_m - a.x_m) * t, a.y_m + (b.y_m - a.y_m) * t,
          a.z_m + (b.z_m - a.z_m) * t};
}

QVariantMap VisualProgramModel::vec_map(const cavr::core::Vec3& v) {
  QVariantMap m;
  m["x"] = v.x_m;
  m["y"] = v.y_m;
  m["z"] = v.z_m;
  return m;
}

cavr::core::Pose3D VisualProgramModel::pose_at(const cavr::core::Vec3& p) {
  return {p, cavr::core::Quaternion::identity()};
}

QVariantMap VisualProgramModel::feature_map(const Feature& f) const {
  QVariantMap m;
  m["index"] = f.index;
  m["id"] = f.id;
  m["name"] = f.name;
  m["kind"] = f.kind;
  m["selected"] = f.index == selected_feature_;
  m["hasOperation"] = std::any_of(operations_.begin(), operations_.end(), [&](const Operation& op) {
    return op.feature_index == f.index;
  });
  m["sx"] = f.start.x_m;
  m["sy"] = f.start.y_m;
  m["sz"] = f.start.z_m;
  m["ex"] = f.end.x_m;
  m["ey"] = f.end.y_m;
  m["ez"] = f.end.z_m;
  m["cx"] = f.center.x_m;
  m["cy"] = f.center.y_m;
  m["cz"] = f.center.z_m;
  m["length"] = distance(f.start, f.end);
  m["width"] = f.width_m;
  m["yaw"] = std::atan2(f.end.y_m - f.start.y_m, f.end.x_m - f.start.x_m) * 180.0 / kPi;
  return m;
}

QVariantMap VisualProgramModel::operation_map(const Operation& op) const {
  QVariantMap m;
  m["index"] = op.index;
  m["featureIndex"] = op.feature_index;
  m["stage"] = op.stage;
  m["name"] = op.name;
  m["type"] = op.params.type;
  m["speed"] = op.params.speed_mm_s;
  m["torchAngle"] = op.params.torch_angle_deg;
  m["standoff"] = op.params.standoff_mm;
  m["approach"] = op.params.approach_mm;
  m["retract"] = op.params.retract_mm;
  m["direction"] = op.params.direction;
  m["weldMode"] = op.params.weld_mode;
  m["duration"] = op.estimated_s;
  m["selected"] = op.index == selected_operation_;
  return m;
}

QVariantMap VisualProgramModel::zone_map(const Zone& z) const {
  QVariantMap m;
  m["index"] = z.index;
  m["id"] = z.id;
  m["name"] = z.name;
  m["type"] = z.type;
  m["x"] = z.center.x_m;
  m["y"] = z.center.y_m;
  m["z"] = z.center.z_m;
  m["sx"] = z.size.x_m;
  m["sy"] = z.size.y_m;
  m["sz"] = z.size.z_m;
  m["color"] = z.color;
  return m;
}

QVariantMap VisualProgramModel::diagnostic_map(const Diagnostic& d) const {
  return base_map(d.severity, d.code, d.message, d.feature_index, d.position);
}

QVariantMap VisualProgramModel::segment_map(const QString& label, const QString& phase,
                                            const cavr::core::Vec3& a,
                                            const cavr::core::Vec3& b,
                                            const QString& color, bool selected) const {
  QVariantMap m;
  const cavr::core::Vec3 c = lerp(a, b, 0.5);
  m["label"] = label;
  m["phase"] = phase;
  m["x"] = c.x_m;
  m["y"] = c.y_m;
  m["z"] = c.z_m;
  m["length"] = std::max(0.01, distance(a, b));
  m["yaw"] = std::atan2(b.y_m - a.y_m, b.x_m - a.x_m) * 180.0 / kPi;
  m["pitch"] = std::atan2(b.z_m - a.z_m,
                          std::hypot(b.x_m - a.x_m, b.y_m - a.y_m)) * 180.0 / kPi;
  m["color"] = color;
  m["selected"] = selected;
  return m;
}

std::vector<cavr::machine::MotionCommand> VisualProgramModel::build_commands(
    const Feature& feature, const OperationParams& params) const {
  const cavr::core::Vec3 weld_start = params.direction == "Reverse" ? feature.end : feature.start;
  const cavr::core::Vec3 weld_end = params.direction == "Reverse" ? feature.start : feature.end;
  const cavr::core::Vec3 approach{weld_start.x_m, weld_start.y_m,
                                  weld_start.z_m + params.approach_mm / 1000.0};
  const cavr::core::Vec3 retract{weld_end.x_m, weld_end.y_m,
                                 weld_end.z_m + params.retract_mm / 1000.0};

  std::vector<cavr::machine::MotionCommand> commands;
  auto move = [](const QString& label, const cavr::core::Vec3& p, double speed) {
    cavr::machine::MotionCommand cmd;
    cmd.kind = cavr::machine::MotionKind::MoveL;
    cmd.target.pose = pose_at(p);
    cmd.speed = speed;
    cmd.blend_radius_m = 0.005;
    cmd.label = label.toStdString();
    return cmd;
  };

  commands.push_back(move("move | safe approach", approach, std::max(250.0, params.speed_mm_s)));
  commands.push_back(move("approach | " + feature.name, weld_start, std::min(120.0, params.speed_mm_s)));

  if (params.type.contains("Weld")) {
    cavr::machine::MotionCommand on;
    on.kind = cavr::machine::MotionKind::ToolOn;
    on.label = ("weld on | " + params.weld_mode).toStdString();
    commands.push_back(std::move(on));
  }

  auto process = move(params.type.toLower() + " | " + feature.name, weld_end, params.speed_mm_s);
  if (params.type.contains("Weld")) {
    cavr::machine::WeldPass pass;
    pass.enabled = true;
    pass.travel_speed_mm_s = params.speed_mm_s;
    pass.tolerance_mm = 0.8;
    pass.segment_length_mm = 2.5;
    pass.process_program = params.weld_mode.toStdString();
    process.weld = pass;
  }
  commands.push_back(std::move(process));

  if (params.type.contains("Weld")) {
    cavr::machine::MotionCommand off;
    off.kind = cavr::machine::MotionKind::ToolOff;
    off.label = "weld off";
    commands.push_back(std::move(off));
  }

  commands.push_back(move("retract | safe exit", retract, std::max(250.0, params.speed_mm_s)));
  return commands;
}

void VisualProgramModel::validate_current() {
  diagnostics_.clear();
  if (features_.empty()) return;
  const Feature* f = selected_feature_ptr();
  if (!f) {
    diagnostics_.push_back({"error", "GEOMETRY_NONE",
                            "No usable geometry is selected. Pick a line, edge, face, or point cloud segment.",
                            -1, {}});
    return;
  }

  Operation preview;
  preview.index = -1;
  preview.feature_index = f->index;
  preview.name = "Preview";
  preview.params = params_;
  preview.start = params_.direction == "Reverse" ? f->end : f->start;
  preview.end = params_.direction == "Reverse" ? f->start : f->end;
  validate_operation(preview);

  for (const auto& op : operations_) validate_operation(op);
}

void VisualProgramModel::validate_operation(const Operation& op) {
  const cavr::core::Vec3 mid = lerp(op.start, op.end, 0.5);
  const double reach = std::sqrt(mid.x_m * mid.x_m + mid.y_m * mid.y_m + mid.z_m * mid.z_m);
  if (reach > 1.45 || mid.z_m < 0.08) {
    diagnostics_.push_back({"error", "REACH",
                            "Target may be outside the GP25 reachable workspace.",
                            op.feature_index, mid});
  }
  if (op.params.standoff_mm < 6.0) {
    diagnostics_.push_back({"error", "TOOL_STANDOFF",
                            "Torch standoff is below 6 mm; collision with the part is likely.",
                            op.feature_index, mid});
  } else if (op.params.standoff_mm > 30.0) {
    diagnostics_.push_back({"warning", "TOOL_STANDOFF",
                            "Torch standoff is high for welding; bead quality may be unstable.",
                            op.feature_index, mid});
  }
  if (std::abs(op.params.torch_angle_deg) > 45.0) {
    diagnostics_.push_back({"warning", "TOOL_ANGLE",
                            "Torch angle is aggressive. Check access and shielding gas coverage.",
                            op.feature_index, mid});
  }
  if (distance(op.start, op.end) < 0.05) {
    diagnostics_.push_back({"error", "GEOMETRY_SHORT",
                            "Selected contour is too short for a stable programmed operation.",
                            op.feature_index, mid});
  }
  if (op.params.speed_mm_s > 650.0 && op.params.type.contains("Weld")) {
    diagnostics_.push_back({"warning", "PROCESS_SPEED",
                            "Weld travel speed is high for this process window.",
                            op.feature_index, mid});
  }

  for (const auto& zone : zones_) {
    const bool hit = inside_box(op.start, zone.center, zone.size) ||
                     inside_box(op.end, zone.center, zone.size) ||
                     inside_box(mid, zone.center, zone.size);
    if (!hit) continue;
    if (zone.type == "forbidden") {
      diagnostics_.push_back({"error", "ZONE_FORBIDDEN",
                              "Path intersects a forbidden clamp volume.",
                              op.feature_index, mid});
    } else if (zone.type == "slow" && op.params.speed_mm_s > 180.0) {
      diagnostics_.push_back({"warning", "ZONE_SLOW",
                              "Path crosses a slowdown zone; reduce approach or weld speed.",
                              op.feature_index, mid});
    }
  }
}
