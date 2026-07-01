# cavr-convert

Converts a recording between storage backends — the authoritative **MCAP** store
and the dependency-free **JSON** reference backend — by loading it into the
neutral `record::Recording` model and writing it back through the other backend.
The message stream is preserved exactly (channels, payloads, log times); only the
container format changes.

```bash
cavr-convert session.mcap session.json   # MCAP -> JSON
cavr-convert session.json session.mcap   # JSON -> MCAP
```

The extension of each path selects the backend (`.mcap` or `.json`). Channel ids
are remapped on the way through (a backend may assign its own), so a round-trip
`.json → .mcap → .json` reproduces the same logical channels and messages.

Built on [`record/copy.hpp`](../../libs/record/include/cavr/record/copy.hpp)'s
`write_recording`, which is the backend-agnostic core of the conversion.
