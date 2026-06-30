# Vendored: MCAP C++ library

This is the upstream header-only C++ implementation of [MCAP](https://mcap.dev),
vendored so CAVR can read and write spec-conformant `.mcap` files that interoperate
with the wider MCAP tooling (Foxglove Studio, the `mcap` CLI, the Python/Go/Rust
readers, ROS tooling).

- **Source:** https://github.com/foxglove/mcap (`cpp/mcap/include/mcap/`)
- **Commit:** `563594c4daa5a6fbf43e00f3342105a76499c538`
- **License:** MIT — see [LICENSE](LICENSE).

Only the headers under `include/mcap/` are vendored. The library is consumed with
compression disabled (`MCAP_COMPRESSION_NO_LZ4`, `MCAP_COMPRESSION_NO_ZSTD`), so it
pulls **no** external dependencies (no lz4, no zstd). The implementation is
compiled in exactly one translation unit in `libs/storage_mcap` via
`MCAP_IMPLEMENTATION`.

This is CAVR's only third-party dependency. It is deliberately isolated behind the
`cavr::record::RecordingWriter` / `RecordingReader` interfaces: nothing in the
project depends on MCAP directly, and the whole backend is gated behind the
`CAVR_ENABLE_MCAP` CMake option.

To update: re-fetch the same file list from a newer upstream commit and update the
commit hash above.
