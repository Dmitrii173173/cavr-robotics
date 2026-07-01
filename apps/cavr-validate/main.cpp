// cavr-validate: runs the pre-execution trajectory validation of the reference
// welding plan against a machine profile — the same check CAVR Studio's Validate
// workflow phase performs — and reports the result. With no argument it uses the
// built-in GP25 cell profile; given a path it validates against a profile loaded
// from JSON. Exits non-zero when validation finds errors, in the manner of a
// linter, so it composes into scripts and CI.

#include <cavr/adapters/mock_robot/mock_controller.hpp>
#include <cavr/machine/machine_profile.hpp>
#include <cavr/machine/profile_io.hpp>
#include <cavr/runtime/demo_plan.hpp>
#include <cavr/validation/trajectory_validator.hpp>

#include <iostream>
#include <string>

namespace {

namespace machine = cavr::machine;
namespace validation = cavr::validation;
namespace runtime = cavr::runtime;

void print_usage() {
  std::cout <<
      "Usage: cavr-validate [<profile.json>]\n\n"
      "Validates the reference welding plan against a machine profile (joint\n"
      "limits, speeds, axis count, referenced frames), the same check the Studio\n"
      "Validate phase runs. With no argument the built-in GP25 cell profile is\n"
      "used. Exits 1 when validation finds errors, 0 otherwise.\n";
}

}  // namespace

int main(int argc, char** argv) {
  if (argc == 2 && (std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h")) {
    print_usage();
    return 0;
  }
  if (argc > 2) {
    print_usage();
    return 2;
  }

  machine::MachineProfile profile;
  if (argc == 2) {
    auto loaded = machine::load_profile(argv[1]);
    if (!loaded.errors.empty()) {
      for (const auto& error : loaded.errors) std::cerr << "error: " << error << '\n';
      return 1;
    }
    profile = std::move(loaded.profile);
    std::cout << "Validating reference welding plan against profile '" << profile.display_name
              << "' (" << argv[1] << ")\n";
  } else {
    profile = cavr::adapters::mock_robot::make_gp25_profile();
    std::cout << "Validating reference welding plan against built-in profile '" << profile.display_name
              << "'\n";
  }

  const machine::MotionTask task = runtime::make_demo_plan().to_motion_task();
  const validation::ValidationReport report = validation::validate_task(profile, task);

  std::size_t warnings = 0;
  for (const auto& issue : report.issues) {
    const char* label = "info";
    switch (issue.severity) {
      case machine::Severity::Warning: label = "warning"; ++warnings; break;
      case machine::Severity::Error: label = "error"; break;
      case machine::Severity::Critical: label = "critical"; break;
      default: break;
    }
    std::cout << "  [" << label << "] ";
    if (issue.step_index >= 0) std::cout << "step " << issue.step_index << ": ";
    std::cout << issue.message << '\n';
  }

  std::cout << "\n" << report.error_count() << " error(s), " << warnings << " warning(s). "
            << "Collisions: " << (report.collisions_evaluated ? "evaluated" : "not evaluated") << ".\n";

  if (!report.ok()) {
    std::cout << "VALIDATION FAILED\n";
    return 1;
  }
  std::cout << "VALIDATION PASSED\n";
  return 0;
}
