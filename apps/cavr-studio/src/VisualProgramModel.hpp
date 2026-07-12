#pragma once

#include <cavr/core/geometry.hpp>
#include <cavr/machine/motion.hpp>

#include <QString>
#include <QVariantList>
#include <QVariantMap>

#include <vector>

class VisualProgramModel final {
 public:
  struct BuildResult final {
    bool ok{false};
    QString message;
    std::vector<cavr::machine::MotionCommand> commands;
  };

  VisualProgramModel();

  void load_demo_workpiece();
  void clear_operations();
  bool select_feature(int index);
  bool select_operation(int index);

  void set_operation_params(const QString& type, double speed_mm_s, double torch_angle_deg,
                            double standoff_mm, double approach_mm, double retract_mm,
                            const QString& direction, const QString& weld_mode);

  [[nodiscard]] int selected_feature() const noexcept { return selected_feature_; }
  [[nodiscard]] int selected_operation() const noexcept { return selected_operation_; }
  [[nodiscard]] QVariantMap operation_params() const;
  [[nodiscard]] QVariantList features() const;
  [[nodiscard]] QVariantList zones() const;
  [[nodiscard]] QVariantList point_cloud() const;
  [[nodiscard]] QVariantList operations() const;
  [[nodiscard]] QVariantList path_segments() const;
  [[nodiscard]] QVariantList diagnostics() const;
  [[nodiscard]] cavr::machine::MotionTask compiled_task() const;

  [[nodiscard]] BuildResult create_operation_from_selection();

 private:
  struct Feature final {
    int index{};
    QString id;
    QString name;
    QString kind;
    cavr::core::Vec3 start{};
    cavr::core::Vec3 end{};
    cavr::core::Vec3 center{};
    cavr::core::Vec3 normal{0.0, 0.0, 1.0};
    double width_m{0.02};
    bool selectable{true};
  };

  struct Zone final {
    int index{};
    QString id;
    QString name;
    QString type;
    cavr::core::Vec3 center{};
    cavr::core::Vec3 size{};
    QString color;
  };

  struct OperationParams final {
    QString type{"Weld seam"};
    double speed_mm_s{120.0};
    double torch_angle_deg{15.0};
    double standoff_mm{12.0};
    double approach_mm{80.0};
    double retract_mm{90.0};
    QString direction{"Forward"};
    QString weld_mode{"Pulse MIG"};
  };

  struct Operation final {
    int index{};
    int feature_index{};
    QString stage;
    QString name;
    OperationParams params;
    cavr::core::Vec3 start{};
    cavr::core::Vec3 end{};
    double estimated_s{};
  };

  struct Diagnostic final {
    QString severity;
    QString code;
    QString message;
    int feature_index{-1};
    cavr::core::Vec3 position{};
  };

  [[nodiscard]] const Feature* selected_feature_ptr() const;
  [[nodiscard]] const Feature* feature_by_index(int index) const;
  [[nodiscard]] static double distance(const cavr::core::Vec3& a, const cavr::core::Vec3& b);
  [[nodiscard]] static cavr::core::Vec3 lerp(const cavr::core::Vec3& a,
                                             const cavr::core::Vec3& b, double t);
  [[nodiscard]] static QVariantMap vec_map(const cavr::core::Vec3& v);
  [[nodiscard]] static cavr::core::Pose3D pose_at(const cavr::core::Vec3& p);
  [[nodiscard]] QVariantMap feature_map(const Feature& f) const;
  [[nodiscard]] QVariantMap operation_map(const Operation& op) const;
  [[nodiscard]] QVariantMap zone_map(const Zone& z) const;
  [[nodiscard]] QVariantMap diagnostic_map(const Diagnostic& d) const;
  [[nodiscard]] QVariantMap segment_map(const QString& label, const QString& phase,
                                        const cavr::core::Vec3& a,
                                        const cavr::core::Vec3& b,
                                        const QString& color,
                                        bool selected) const;
  [[nodiscard]] std::vector<cavr::machine::MotionCommand> build_commands(
      const Feature& feature, const OperationParams& params) const;
  void validate_current();
  void validate_operation(const Operation& op);

  std::vector<Feature> features_;
  std::vector<Zone> zones_;
  std::vector<cavr::core::Vec3> point_cloud_;
  std::vector<Operation> operations_;
  std::vector<Diagnostic> diagnostics_;
  OperationParams params_;
  int selected_feature_{0};
  int selected_operation_{-1};
};
