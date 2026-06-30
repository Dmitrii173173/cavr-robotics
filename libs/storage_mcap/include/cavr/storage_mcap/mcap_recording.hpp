#pragma once

// MCAP-backed implementation of the storage-neutral recording interfaces. Writing
// produces spec-conformant .mcap files that interoperate with the wider MCAP
// ecosystem (Foxglove Studio, the mcap CLI, the Python/Go/Rust readers).
//
// The vendored MCAP library is fully isolated: it is included and compiled in a
// single translation unit (mcap_recording.cpp) and never leaks through this
// header. Consumers depend only on cavr::record and the standard library, exactly
// as they do for the JSON reference backend — MCAP is a drop-in alternative.

#include <cavr/record/reader.hpp>
#include <cavr/record/writer.hpp>

#include <filesystem>
#include <memory>
#include <string_view>
#include <vector>

namespace cavr::storage_mcap {

// Writes a recording as an uncompressed .mcap file. Channels are registered up
// front; messages are appended and the file is finalized on close().
class McapRecordingWriter final : public record::RecordingWriter {
 public:
  // When `streaming` is true the file is written without chunking, so each message
  // is appended to the data section as it is written (incremental, no buffered
  // chunk) — suited to a live recorder. When false (default) the writer chunks for
  // a compact, index-friendly file better suited to post-hoc writes.
  explicit McapRecordingWriter(const std::filesystem::path& path, bool streaming = false);
  ~McapRecordingWriter() override;

  record::ChannelId add_channel(const record::Channel& channel) override;
  record::RecordStatus write(const record::Message& message) override;
  record::RecordStatus close() override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// Result of loading an .mcap file into the neutral in-memory model.
struct McapRecordingLoad final {
  record::Recording recording;
  record::RecordStatus status;
};

// Reads an .mcap file into a Recording. On failure `recording` is empty and
// `status` carries the reason. Channels that carry no messages are not
// reconstructed (the message stream is the source of truth here).
[[nodiscard]] McapRecordingLoad load_recording(const std::filesystem::path& path);

// Reader serving a Recording loaded from an .mcap file. Construct it from the
// data returned by load_recording(); the query surface is pure cavr::record and
// needs none of the MCAP machinery.
class McapRecordingReader final : public record::RecordingReader {
 public:
  explicit McapRecordingReader(record::Recording recording) : recording_(std::move(recording)) {}

  const std::vector<record::Channel>& channels() const override { return recording_.channels; }

  const record::Channel* find_channel(std::string_view topic) const override {
    return recording_.find_channel(topic);
  }

  std::vector<record::Message> messages() const override { return recording_.sorted_by_time(); }

  std::vector<record::Message> messages_on(record::ChannelId channel) const override {
    return recording_.on_channel(channel);
  }

 private:
  record::Recording recording_;
};

}  // namespace cavr::storage_mcap
