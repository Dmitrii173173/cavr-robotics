#pragma once

// Builds a catalog row by scanning a recording — the bridge that keeps the catalog
// reconstructible: re-open any recording and rebuild its metadata. It reads the
// session header (id, profile, span) and the camera channel through the neutral
// RecordingReader, so it works for any backend (JSON or MCAP). File facts (path,
// size, content hash) are filesystem properties the caller supplies, since they
// are not part of the message stream.

#include <cavr/catalog/catalog.hpp>
#include <cavr/record/reader.hpp>
#include <cavr/runtime/camera_recording.hpp>
#include <cavr/runtime/record_session.hpp>
#include <cavr/validation/trajectory_validator.hpp>

#include <cstdint>
#include <string>
#include <utility>

namespace cavr::runtime {

namespace validation = cavr::validation;

// Summarizes a validation run for the catalog: outcome, issue counts and the
// first issue's message, plus the honest "collisions not evaluated" flag.
[[nodiscard]] inline catalog::ValidationSummary to_validation_summary(
    const validation::ValidationReport& report, std::int64_t created_ns = 0) {
  catalog::ValidationSummary summary;
  summary.passed = report.ok();
  summary.error_count = static_cast<int>(report.error_count());
  summary.collisions_evaluated = report.collisions_evaluated;
  summary.created_ns = created_ns;
  for (const auto& issue : report.issues) {
    if (issue.severity == machine::Severity::Warning) ++summary.warning_count;
  }
  if (!report.issues.empty()) summary.detail = report.issues.front().message;
  return summary;
}

[[nodiscard]] inline catalog::CatalogSession index_recording(const record::RecordingReader& reader,
                                                             std::string file_path,
                                                             std::uint64_t file_size,
                                                             std::string content_hash) {
  catalog::CatalogSession entry;

  const SessionRecordingResult session = read_session(reader);
  entry.id = session.log.session_id;
  entry.name = session.log.profile.display_name;
  entry.start_ns = session.log.started.nanoseconds();
  entry.duration_ns = (session.log.ended - session.log.started).nanoseconds();
  entry.robot_model = session.log.profile.robot_model;

  const auto cameras = read_camera_frames(reader);
  if (!cameras.empty()) {
    entry.camera_model = cameras.front().frame_id + " (" + cameras.front().encoding + ")";
  }

  entry.file_path = std::move(file_path);
  entry.file_size = file_size;
  entry.content_hash = std::move(content_hash);
  return entry;
}

}  // namespace cavr::runtime
