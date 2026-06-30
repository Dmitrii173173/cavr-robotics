# Runtime

The workflow/session layer that ties configuration, adapters and validation into
the CAVR Studio production flow:

```
connect -> use/discover MachineProfile -> set plan -> validate -> execute -> tick* -> replay
```

- **`timeline.hpp`** — `OperationStep` / `TimelineEvent` / `Timeline`: the plan
  (scan, calibration, capture, planning, validation, motion, welding, IO, …) and
  the events recorded while it runs.
- **`session_manager.hpp`** — `SessionManager`: drives a `ControllerAdapter`
  through the workflow and records a `SessionLog`. It never synthesizes motion;
  every frame comes from `adapter.poll()`.
- **`session.hpp`** — `SessionLog` (profile + plan + telemetry) and `ReplayCursor`.
- **`session_io.hpp`** — save/load a session log as a single JSON document for
  replay/diagnostics.
- **`record_session.hpp`** — bridges a `SessionLog` onto the storage-neutral
  recording interfaces (`cavr::record`): telemetry frames, controller events and a
  session header become channelized message streams. `write_session()` persists a
  finished log; `read_session()` is the Replay-phase source — backend-agnostic, so
  the same session persists through the JSON reference backend or MCAP.
- **`session_recorder.hpp`** — `SessionRecorder`: a **live** Monitor-phase sink.
  Attach it to a `SessionManager` (`attach_recorder`) and each `tick()` streams its
  frame to the backend as it arrives, instead of buffering the whole log. Tracks
  session statistics (frames/events/camera frames written, errors); `finish()`
  writes the header and finalizes. With the MCAP backend in streaming mode the file
  is unchunked so messages are appended incrementally.
- **`camera_recording.hpp`** — serializes `CameraFrame`s to/from JSON and reads the
  `camera/color` channel back. A `SessionManager` with `attach_camera()` polls a
  `CameraAdapter` on the same tick as the robot, so a recording carries a
  **synchronized robot + vision** stream; heavy pixel payloads live in the
  authoritative store, never in a metadata catalog.
- **`demo_plan.hpp`** — a representative welding workflow for the demo and tests.

Real execution is delegated to the controller/teach pendant; this layer plans,
validates, hands the task over, and monitors the telemetry that comes back.
