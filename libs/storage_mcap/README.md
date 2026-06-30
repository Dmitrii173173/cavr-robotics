# MCAP Storage

The authoritative recording backend. Implements the storage-neutral
`cavr::record::RecordingWriter` / `RecordingReader` interfaces on top of
[MCAP](https://mcap.dev), so recorded sessions are spec-conformant `.mcap` files
that interoperate with the wider MCAP ecosystem (Foxglove Studio, the `mcap` CLI,
the Python/Go/Rust readers, ROS tooling).

- `McapRecordingWriter` — writes an uncompressed `.mcap` file; a drop-in
  alternative to the JSON reference backend behind the same interface.
- `load_recording()` + `McapRecordingReader` — read an `.mcap` file back into the
  neutral in-memory model for replay/diagnostics.

The vendored MCAP library ([`third_party/mcap`](../../third_party/mcap)) is CAVR's
only third-party dependency and is fully isolated: it is compiled in a single
translation unit (`src/mcap_recording.cpp`) with compression disabled
(`MCAP_COMPRESSION_NO_LZ4`, `MCAP_COMPRESSION_NO_ZSTD`, so no lz4/zstd), and it
never leaks past `mcap_recording.cpp` — consumers see only `cavr::record`.

Gated behind the `CAVR_ENABLE_MCAP` CMake option (default `ON`). With it `OFF`,
the module falls back to a header-only stub and nothing depends on MCAP.

This backend treats message payloads as opaque bytes. Heavy streams (RGB/depth
images, point clouds, high-frequency telemetry) are exactly what MCAP is built to
hold; lightweight, reconstructible metadata belongs in a future catalog index,
not here.

Future work: chunked/streaming writes for long sessions, attachments/metadata
records, and a `SessionLog` ↔ recording bridge wired into the runtime's Monitor
and Replay phases.
