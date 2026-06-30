#pragma once

// Live session recorder: streams telemetry to a recording backend frame by frame
// as it arrives during the Monitor phase, instead of buffering the whole
// SessionLog and serializing it once at the end. Each frame is handed to the
// backend immediately, so the recording grows incrementally and a long session no
// longer has to live entirely in memory before it can be persisted.
//
// Recordings written here are read back by runtime::read_session(), so the live
// path and the post-hoc write_session() path produce interchangeable recordings.
// The session header (id, profile, plan, span) is written on finish(); the
// irreplaceable telemetry stream is written as it happens.

#include <cavr/adapter_sdk/camera_frame.hpp>
#include <cavr/adapter_sdk/robot_state.hpp>
#include <cavr/record/writer.hpp>
#include <cavr/runtime/camera_recording.hpp>
#include <cavr/runtime/record_session.hpp>
#include <cavr/runtime/session.hpp>

#include <cstddef>
#include <string>

namespace cavr::runtime {

class SessionRecorder final {
 public:
  explicit SessionRecorder(record::RecordingWriter& writer) : writer_(&writer) {}

  // Registers the channels. Safe to call more than once; only the first takes
  // effect. Called automatically by the first record_frame()/finish().
  void begin() {
    if (begun_) return;
    meta_ = writer_->add_channel({0, std::string(detail::kSessionMetaTopic),
                                  std::string(record::content_type::kJson),
                                  std::string(detail::kSessionMetaSchema)});
    telemetry_ = writer_->add_channel({0, std::string(record::topics::kRobotTelemetry),
                                       std::string(record::content_type::kJson),
                                       std::string(record::schemas::kRobotState)});
    events_ = writer_->add_channel({0, std::string(record::topics::kControllerEvents),
                                    std::string(record::content_type::kJson),
                                    std::string(record::schemas::kControllerEvent)});
    camera_ = writer_->add_channel({0, std::string(record::topics::kCameraColor),
                                    std::string(record::content_type::kJson),
                                    std::string(detail::kCameraSchema)});
    begun_ = true;
  }

  // Streams one telemetry frame and the controller events it carries. Stops at the
  // first backend error and accounts it.
  record::RecordStatus record_frame(const sdk::RobotState& frame) {
    if (!begun_) begin();

    record::RecordStatus status =
        writer_->write({telemetry_, frame.timestamp, detail::frame_to_json(frame).dump(0)});
    if (!status) {
      ++errors_;
      return status;
    }
    ++frames_written_;

    for (const auto& e : frame.events) {
      const TimelineEvent te{e.timestamp, e.kind, e.severity, e.message, frame.current_step};
      record::RecordStatus event_status =
          writer_->write({events_, e.timestamp, detail::event_to_json(te).dump(0)});
      if (!event_status) {
        ++errors_;
        return event_status;
      }
      ++events_written_;
    }
    return record::RecordStatus::success();
  }

  // Streams one camera frame, time-aligned with the telemetry recorded on the same
  // tick. The (potentially heavy) pixel payload lands in the authoritative store.
  record::RecordStatus record_camera_frame(const sdk::CameraFrame& frame) {
    if (!begun_) begin();
    record::RecordStatus status =
        writer_->write({camera_, frame.timestamp, detail::camera_frame_to_json(frame).dump(0)});
    if (!status) {
      ++errors_;
      return status;
    }
    ++camera_frames_written_;
    return record::RecordStatus::success();
  }

  // Writes the session header and finalizes the recording. Idempotent. The header
  // is stamped at the session end time so writes stay in non-decreasing log-time
  // order for chunked backends.
  record::RecordStatus finish(const SessionLog& log) {
    if (finished_) return record::RecordStatus::success();
    if (!begun_) begin();

    if (record::RecordStatus status =
            writer_->write({meta_, log.ended, detail::session_header_to_json(log).dump(0)});
        !status) {
      ++errors_;
      return status;
    }
    finished_ = true;
    return writer_->close();
  }

  // Session statistics.
  [[nodiscard]] std::size_t frames_written() const noexcept { return frames_written_; }
  [[nodiscard]] std::size_t events_written() const noexcept { return events_written_; }
  [[nodiscard]] std::size_t camera_frames_written() const noexcept { return camera_frames_written_; }
  [[nodiscard]] std::size_t errors() const noexcept { return errors_; }

 private:
  record::RecordingWriter* writer_;
  record::ChannelId meta_{record::kInvalidChannel};
  record::ChannelId telemetry_{record::kInvalidChannel};
  record::ChannelId events_{record::kInvalidChannel};
  record::ChannelId camera_{record::kInvalidChannel};
  std::size_t frames_written_{0};
  std::size_t events_written_{0};
  std::size_t camera_frames_written_{0};
  std::size_t errors_{0};
  bool begun_{false};
  bool finished_{false};
};

}  // namespace cavr::runtime
