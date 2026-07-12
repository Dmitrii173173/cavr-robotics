#include "VisualProgramModel.hpp"

#include <cstdlib>
#include <iostream>
#include <string>

namespace {

void require(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << message << '\n';
    std::exit(1);
  }
}

bool has_error(const QVariantList& diagnostics, const QString& code) {
  for (const QVariant& entry : diagnostics) {
    const QVariantMap d = entry.toMap();
    if (d.value("severity").toString() == "error" && d.value("code").toString() == code) {
      return true;
    }
  }
  return false;
}

}  // namespace

int main() {
  VisualProgramModel model;

  require(!model.features().empty(), "demo workpiece exposes selectable geometry");
  require(!model.zones().empty(), "demo workpiece exposes process zones");
  require(!model.point_cloud().empty(), "demo workpiece exposes a point cloud");

  require(model.select_feature(0), "feature selection succeeds");
  model.set_operation_params("Weld seam", 120.0, 15.0, 12.0, 80.0, 90.0,
                             "Forward", "Pulse MIG");
  auto result = model.create_operation_from_selection();
  require(result.ok, "selected feature compiles into a visual operation");
  require(!result.commands.empty(), "visual operation compiles into motion commands");
  require(!model.operations().empty(), "visual operation is retained in the internal model");
  require(model.compiled_task().size() == result.commands.size(),
          "compiled_task mirrors the retained visual operations");

  model.set_operation_params("Weld seam", 700.0, 60.0, 2.0, 80.0, 90.0,
                             "Forward", "Pulse MIG");
  require(has_error(model.diagnostics(), "TOOL_STANDOFF"),
          "unsafe standoff is reported as a blocking diagnostic");

  model.clear_operations();
  require(model.operations().empty(), "clear_operations removes visual operations");
  require(model.compiled_task().empty(), "clear_operations removes compiled task output");

  return 0;
}
