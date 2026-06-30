#pragma once

// Backend-neutral recording writer. A concrete backend (the reference JSON
// backend today, MCAP later) implements this interface; producers such as the
// runtime's Monitor phase depend only on the interface. Channels are registered
// up front, then messages are appended and the recording is finalized on close.

#include <cavr/record/recording.hpp>

namespace cavr::record {

class RecordingWriter {
 public:
  RecordingWriter() = default;
  virtual ~RecordingWriter() = default;

  RecordingWriter(const RecordingWriter&) = delete;
  RecordingWriter& operator=(const RecordingWriter&) = delete;

  // Registers a channel and returns the id to stamp on its messages.
  virtual ChannelId add_channel(const Channel& channel) = 0;

  // Appends one message. Fails if the channel id was never registered or the
  // writer is already closed.
  virtual RecordStatus write(const Message& message) = 0;

  // Flushes and finalizes the recording. Idempotent.
  virtual RecordStatus close() = 0;
};

}  // namespace cavr::record
