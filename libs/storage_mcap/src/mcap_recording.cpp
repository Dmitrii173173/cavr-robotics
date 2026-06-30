// The single translation unit that compiles the vendored MCAP library. The
// implementation macros are defined here and nowhere else, and the MCAP headers
// are included here only, so the dependency never leaks past this file.

#define MCAP_IMPLEMENTATION  // compile the vendored MCAP library here only
// MCAP_COMPRESSION_NO_LZ4 / MCAP_COMPRESSION_NO_ZSTD are supplied by the build so
// the library pulls no compression dependencies.

#include <cavr/storage_mcap/mcap_recording.hpp>

#include <mcap/reader.hpp>
#include <mcap/writer.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <map>
#include <string>

namespace cavr::storage_mcap {

namespace {

[[nodiscard]] std::uint64_t to_u64_ns(core::Timestamp t) {
  const auto ns = t.nanoseconds();
  return ns < 0 ? 0 : static_cast<std::uint64_t>(ns);
}

}  // namespace

// ---------------------------------------------------------------- writer

struct McapRecordingWriter::Impl {
  mcap::McapWriter writer;
  bool opened{false};
  bool closed{false};
  std::string open_error;
  std::map<record::ChannelId, std::uint32_t> sequence;  // per-channel message counter
};

McapRecordingWriter::McapRecordingWriter(const std::filesystem::path& path, bool streaming)
    : impl_(std::make_unique<Impl>()) {
  mcap::McapWriterOptions options("cavr");
  options.compression = mcap::Compression::None;
  options.noChunking = streaming;  // append each message immediately for live recording
  const mcap::Status status = impl_->writer.open(path.string(), options);
  if (status.ok()) {
    impl_->opened = true;
  } else {
    impl_->open_error = "Failed to open MCAP file for writing: " + status.message;
  }
}

McapRecordingWriter::~McapRecordingWriter() {
  if (impl_ && impl_->opened && !impl_->closed) impl_->writer.close();
}

record::ChannelId McapRecordingWriter::add_channel(const record::Channel& channel) {
  if (!impl_->opened || impl_->closed) return record::kInvalidChannel;

  mcap::Schema schema(channel.schema_name, "", "");
  impl_->writer.addSchema(schema);

  mcap::Channel mcap_channel(channel.topic, channel.content_type, schema.id);
  impl_->writer.addChannel(mcap_channel);

  // MCAP assigns channel ids; reuse them directly as our ChannelId (both uint16).
  const auto id = static_cast<record::ChannelId>(mcap_channel.id);
  impl_->sequence.emplace(id, 0);
  return id;
}

record::RecordStatus McapRecordingWriter::write(const record::Message& message) {
  if (!impl_->opened) return record::RecordStatus::failure(impl_->open_error);
  if (impl_->closed) return record::RecordStatus::failure("MCAP writer is already closed");

  auto seq = impl_->sequence.find(message.channel_id);
  if (seq == impl_->sequence.end()) {
    return record::RecordStatus::failure("Message references unknown channel id " +
                                         std::to_string(message.channel_id));
  }

  mcap::Message mcap_message;
  mcap_message.channelId = static_cast<mcap::ChannelId>(message.channel_id);
  mcap_message.sequence = seq->second++;
  mcap_message.logTime = to_u64_ns(message.log_time);
  mcap_message.publishTime = mcap_message.logTime;
  mcap_message.data = reinterpret_cast<const std::byte*>(message.data.data());
  mcap_message.dataSize = message.data.size();

  const mcap::Status status = impl_->writer.write(mcap_message);
  if (!status.ok()) return record::RecordStatus::failure("MCAP write failed: " + status.message);
  return record::RecordStatus::success();
}

record::RecordStatus McapRecordingWriter::close() {
  if (impl_->closed) return record::RecordStatus::success();
  if (!impl_->opened) return record::RecordStatus::failure(impl_->open_error);
  impl_->writer.close();
  impl_->closed = true;
  return record::RecordStatus::success();
}

// ---------------------------------------------------------------- reader

McapRecordingLoad load_recording(const std::filesystem::path& path) {
  McapRecordingLoad result;

  mcap::McapReader reader;
  const mcap::Status open_status = reader.open(path.string());
  if (!open_status.ok()) {
    result.status = record::RecordStatus::failure("Failed to open MCAP file: " + open_status.message);
    return result;
  }

  std::map<record::ChannelId, record::Channel> channels;  // ordered by id, deduplicated

  mcap::ProblemCallback on_problem = [&](const mcap::Status& problem) {
    if (result.status.ok) result.status = record::RecordStatus::failure("MCAP read problem: " + problem.message);
  };

  auto view = reader.readMessages(on_problem);
  for (auto it = view.begin(); it != view.end(); ++it) {
    const mcap::Channel* channel = it->channel.get();
    if (channel == nullptr) continue;

    const auto id = static_cast<record::ChannelId>(channel->id);
    if (channels.find(id) == channels.end()) {
      record::Channel c;
      c.id = id;
      c.topic = channel->topic;
      c.content_type = channel->messageEncoding;
      c.schema_name = it->schema ? it->schema->name : std::string{};
      channels.emplace(id, std::move(c));
    }

    record::Message m;
    m.channel_id = id;
    m.log_time = core::Timestamp::from_nanoseconds(static_cast<core::Timestamp::rep>(it->message.logTime));
    m.data.assign(reinterpret_cast<const char*>(it->message.data), it->message.dataSize);
    result.recording.messages.push_back(std::move(m));
  }
  reader.close();

  for (auto& [id, channel] : channels) result.recording.channels.push_back(std::move(channel));

  if (result.status.ok) result.status = record::RecordStatus::success();
  return result;
}

}  // namespace cavr::storage_mcap
