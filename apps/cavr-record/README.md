# cavr-record

Runs the demo GP25 welding workflow against the mock robot and camera adapters
and streams the resulting synchronized robot + vision session live to a
recording, using the same `SessionManager` + `SessionRecorder` path CAVR
Studio's Monitor phase uses. Exercises the record → storage (→ catalog)
backend end to end from the command line.

```bash
cavr-record --out session.mcap --session-id demo_0 --catalog catalog.db
```

- `--out <path>` — output recording; `.mcap` (MCAP, default when built) or
  `.json` (dependency-free reference backend).
- `--session-id <id>` — defaults to a timestamped id.
- `--catalog <path>` — also indexes the finished recording into a catalog at
  this path (SQLite when built, otherwise an in-memory catalog that does not
  persist — useful to prove the indexing path even without SQLite).
- `--ticks <n>` — safety cap on simulated ticks (default 5000).

Run `cavr-record --help` for the full option list.
