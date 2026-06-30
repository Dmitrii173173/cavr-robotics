# Catalog

A small **local catalog** of recorded sessions for browsing and search — the first
database layer in CAVR. It stores only lightweight, **reconstructible** metadata:
per session the id, name, recording path, start, duration, robot/camera model,
calibration id, file size and content hash; plus **tags**, **annotations**
(time-anchored or session-level notes), **bookmarks** (named instants) and
**validation run summaries** (outcome + issue counts). It never holds telemetry,
images or point clouds.

The boundary is deliberate: the authoritative data lives in the **MCAP** recordings
the catalog points at. Deleting or rebuilding the catalog never touches session
data, and the catalog can be rebuilt by re-scanning the recording files.

- `catalog.hpp` — domain types (`CatalogSession`, `SessionId`, `CatalogStatus`),
  the engine-neutral `Catalog` interface, the schema version, and a file
  fingerprint (size + FNV-1a hash) helper.
- `in_memory_catalog.hpp` — dependency-free reference implementation; pins the
  semantics and serves as the fallback when SQLite is disabled.
- `sqlite_catalog.hpp` / `src/sqlite_catalog.cpp` — the persistent **SQLite**
  backend. The vendored amalgamation ([`third_party/sqlite`](../../third_party/sqlite))
  is compiled in this one translation unit and never leaks past it (the `sqlite3`
  handle lives behind a PIMPL). On `initialize()` it creates the schema and rejects
  a catalog written by a newer schema version.

Indexing a recording into a `CatalogSession` is done by
[`runtime/catalog_index.hpp`](../runtime/include/cavr/runtime/catalog_index.hpp),
which scans any recording (JSON or MCAP) through the neutral `RecordingReader`.

Gated behind the `CAVR_ENABLE_SQLITE` CMake option (default `ON`). With it `OFF`,
only the header-only types and in-memory catalog are built and nothing depends on
SQLite.
