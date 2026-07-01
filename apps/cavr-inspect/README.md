# cavr-inspect

Dumps the contents of a recording through the storage-neutral `RecordingReader`:
channels with their message counts, the session header (id, robot model, span,
frame/event counts) and the camera stream, if present.

```bash
cavr-inspect session.mcap
```

Works on both `.mcap` and `.json` recordings — the same tool regardless of
which backend wrote the file.
