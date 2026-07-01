#pragma once

// Copies a whole in-memory Recording through any RecordingWriter, remapping
// channel ids as it goes. A writer is free to assign its own channel ids
// (the MCAP backend does; the JSON backend preserves them), so message channel
// references cannot be replayed verbatim — each source channel is registered
// with the destination writer and its returned id is substituted on every
// message that referenced it. This is the backend-agnostic core of format
// conversion: load a Recording from one backend, write it through another.

#include <cavr/record/recording.hpp>
#include <cavr/record/writer.hpp>

#include <string>
#include <unordered_map>

namespace cavr::record {

// Registers every channel of `src` with `dst`, then writes every message in
// log_time order under its remapped channel id, and finalizes `dst`. Stops and
// returns the first failure (a message on an unregistered channel, or a writer
// error); on success the destination is closed.
[[nodiscard]] inline RecordStatus write_recording(const Recording& src, RecordingWriter& dst) {
  std::unordered_map<ChannelId, ChannelId> id_map;
  for (const auto& channel : src.channels) {
    id_map[channel.id] = dst.add_channel(channel);
  }

  for (const auto& message : src.sorted_by_time()) {
    const auto it = id_map.find(message.channel_id);
    if (it == id_map.end()) {
      return RecordStatus::failure("Message references unknown channel id " +
                                   std::to_string(message.channel_id));
    }
    Message remapped = message;
    remapped.channel_id = it->second;
    if (RecordStatus status = dst.write(remapped); !status) return status;
  }

  return dst.close();
}

}  // namespace cavr::record
