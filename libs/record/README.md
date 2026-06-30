# Record

Storage-neutral recording layer: the seam between data producers (the runtime's
Monitor phase) and whatever stores the data on disk.

A recording is a set of typed **Channels** and a stream of timestamped
**Messages**, each carrying an opaque, already-serialized payload. The vocabulary
deliberately mirrors MCAP (channel + schema + message with `log_time` and opaque
bytes), so the authoritative MCAP backend can later implement the same interfaces
as a drop-in replacement.

- `recording.hpp` — `Channel`, `Message`, `ChannelId`, `RecordStatus`, the
  well-known topics/schemas, and the in-memory `Recording` container
  (channel-id assignment, topic lookup, time ordering).
- `writer.hpp` / `reader.hpp` — the backend-neutral `RecordingWriter` and
  `RecordingReader` interfaces. Producers and consumers depend only on these.
- `json_recording.hpp` — the **reference backend**: serializes a recording to the
  project's hand-rolled JSON (`cavr::json`) and reads it back, with explicit
  format versioning (`kRecordingFormatVersion`; newer files are rejected).

This layer never decodes payloads. Heavy streams — RGB/depth images, point
clouds, high-frequency telemetry — belong in the authoritative store (MCAP);
camera channels here are reserved for references/metadata only.

Future work (later modules): an MCAP-backed `RecordingWriter`/`RecordingReader`
in `libs/storage_mcap`, streaming writes, source coordination, bounded queues,
dropped-message accounting, and session statistics.
