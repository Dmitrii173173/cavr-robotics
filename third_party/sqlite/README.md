# Vendored: SQLite amalgamation

The single-file [SQLite](https://sqlite.org) amalgamation, vendored so CAVR can
keep a small **local catalog** of session metadata without an external dependency
manager.

- **Source:** https://www.sqlite.org/2026/sqlite-amalgamation-3530300.zip
- **Version:** 3.53.3
- **Files:** `sqlite3.c`, `sqlite3.h` (the shell and extension headers are not
  vendored).
- **License:** SQLite is in the **public domain** — see https://www.sqlite.org/copyright.html.

`sqlite3.c` is compiled in a single translation unit in `libs/catalog`. The
dependency is isolated behind the `cavr::catalog::Catalog` interface (consumers
never include `sqlite3.h`) and gated behind the `CAVR_ENABLE_SQLITE` CMake option.

The catalog stores only lightweight, reconstructible metadata (session id, path,
span, robot/camera model, file size, content hash, tags). The authoritative
recorded data lives in the MCAP files the catalog indexes — deleting or rebuilding
the catalog never touches session data.

To update: re-fetch a newer amalgamation, replace both files, and update the
version above.
