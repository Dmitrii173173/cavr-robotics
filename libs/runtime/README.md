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
- **`session_io.hpp`** — save/load a session log as JSON for replay/diagnostics.
- **`demo_plan.hpp`** — a representative welding workflow for the demo and tests.

Real execution is delegated to the controller/teach pendant; this layer plans,
validates, hands the task over, and monitors the telemetry that comes back.
