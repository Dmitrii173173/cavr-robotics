#pragma once

// Backend-neutral recording reader. Mirrors RecordingWriter: a concrete backend
// implements this, and replay/diagnostics consumers depend only on the interface.
// Messages are returned in log_time order so callers can replay a synchronized
// stream without knowing how it was stored.

#include <cavr/record/recording.hpp>

namespace cavr::record {

class RecordingReader {
 public:
  RecordingReader() = default;
  virtual ~RecordingReader() = default;

  RecordingReader(const RecordingReader&) = delete;
  RecordingReader& operator=(const RecordingReader&) = delete;

  virtual const std::vector<Channel>& channels() const = 0;

  // Returns the channel for a topic, or nullptr when absent.
  virtual const Channel* find_channel(std::string_view topic) const = 0;

  // All messages, ordered by log_time.
  virtual std::vector<Message> messages() const = 0;

  // Messages on one channel, ordered by log_time. An unknown channel id yields
  // an empty result (a controlled, non-error empty stream).
  virtual std::vector<Message> messages_on(ChannelId channel) const = 0;
};

}  // namespace cavr::record
