#pragma once

// Storage-neutral recording model: a recording is a set of typed Channels and a
// stream of timestamped Messages, each carrying an opaque serialized payload.
// This mirrors the channel/message vocabulary of MCAP so a future MCAP backend
// is a drop-in implementation of the writer/reader interfaces, while the rest of
// CAVR depends only on these types and never on a concrete storage engine.
//
// The recording layer deliberately treats Message::data as opaque bytes: it
// never decodes telemetry, events or camera references. Heavy streams (images,
// depth maps, point clouds) belong in the authoritative store (MCAP), not here.

#include <cavr/core/time.hpp>

#include <algorithm>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace cavr::record {

// On-disk/format version for the reference JSON backend. Readers reject any
// recording whose version is newer than they understand (see json_recording.hpp).
inline constexpr int kRecordingFormatVersion = 1;

// Channel identifiers are small and stable within a single recording. 0 means
// "unassigned"; the writer assigns a real id when a channel is registered.
using ChannelId = std::uint16_t;
inline constexpr ChannelId kInvalidChannel = 0;

// A logical stream of like-typed messages (e.g. robot telemetry). content_type
// describes how Message::data is encoded; schema_name names the logical payload
// type. Neither is interpreted by this layer.
struct Channel final {
  ChannelId id{kInvalidChannel};
  std::string topic;
  std::string content_type;
  std::string schema_name;
};

// One timestamped record on a channel. data is an opaque, already-serialized
// payload; this layer never parses it.
struct Message final {
  ChannelId channel_id{kInvalidChannel};
  core::Timestamp log_time{core::Timestamp::zero()};
  std::string data;
};

// Well-known channel topics. Camera channels are reserved here for Phase A and
// are expected to carry references/metadata only; pixel and point-cloud payloads
// land in the authoritative MCAP store, not in the reference backend.
namespace topics {
inline constexpr std::string_view kRobotTelemetry = "robot/telemetry";
inline constexpr std::string_view kControllerEvents = "controller/events";
inline constexpr std::string_view kCameraColor = "camera/color";
}  // namespace topics

namespace content_type {
inline constexpr std::string_view kJson = "application/json";
}  // namespace content_type

namespace schemas {
inline constexpr std::string_view kRobotState = "cavr.adapter_sdk.RobotState";
inline constexpr std::string_view kControllerEvent = "cavr.adapter_sdk.ControllerEvent";
}  // namespace schemas

// Result of a recording operation, in the style of the project's other I/O
// helpers (a boolean outcome plus a human-readable error).
struct RecordStatus final {
  bool ok{true};
  std::string error;

  [[nodiscard]] explicit operator bool() const noexcept { return ok; }

  [[nodiscard]] static RecordStatus success() { return {}; }
  [[nodiscard]] static RecordStatus failure(std::string message) {
    return RecordStatus{false, std::move(message)};
  }
};

// In-memory, storage-neutral representation of a whole recording. Both the
// reference backend and tests build and inspect recordings through this type.
struct Recording final {
  std::vector<Channel> channels;
  std::vector<Message> messages;  // kept in insertion order

  // Registers a channel and returns its id. When channel.id is unassigned a new
  // id (one past the current maximum) is allocated.
  ChannelId add_channel(Channel channel) {
    if (channel.id == kInvalidChannel) {
      ChannelId next = 1;
      for (const auto& c : channels) next = std::max<ChannelId>(next, static_cast<ChannelId>(c.id + 1));
      channel.id = next;
    }
    const ChannelId id = channel.id;
    channels.push_back(std::move(channel));
    return id;
  }

  [[nodiscard]] const Channel* find_channel(std::string_view topic) const {
    for (const auto& c : channels) {
      if (c.topic == topic) return &c;
    }
    return nullptr;
  }

  [[nodiscard]] const Channel* channel_by_id(ChannelId id) const {
    for (const auto& c : channels) {
      if (c.id == id) return &c;
    }
    return nullptr;
  }

  // Messages ordered by log_time. The sort is stable, so messages sharing a
  // timestamp keep their insertion order.
  [[nodiscard]] std::vector<Message> sorted_by_time() const {
    std::vector<Message> out = messages;
    std::stable_sort(out.begin(), out.end(), [](const Message& a, const Message& b) {
      return a.log_time.nanoseconds() < b.log_time.nanoseconds();
    });
    return out;
  }

  // Messages on a single channel, ordered by log_time.
  [[nodiscard]] std::vector<Message> on_channel(ChannelId id) const {
    std::vector<Message> out;
    for (const auto& m : messages) {
      if (m.channel_id == id) out.push_back(m);
    }
    std::stable_sort(out.begin(), out.end(), [](const Message& a, const Message& b) {
      return a.log_time.nanoseconds() < b.log_time.nanoseconds();
    });
    return out;
  }
};

}  // namespace cavr::record
