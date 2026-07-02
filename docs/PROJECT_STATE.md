# CAVR — Project State

A shared snapshot of where CAVR Studio is, so everyone has the same picture.
Keep this updated as the architecture evolves.

_Last updated: 2026-07-01._

## What CAVR is

**CAVR — Calibration-Aware Validation and Replay for Vision-Guided Industrial
Robotics.** A ROS-free C++20 framework + Qt 6 desktop app (**CAVR Studio**) for
connecting to industrial robots/welding cells, configuring them, planning and
validating operations, executing via the controller, and monitoring/replaying
the resulting telemetry. Focus application: vision-guided robotic **welding**.

## Robot asset — Yaskawa GP25

A fully articulated GP25 is the first machine asset.

- **Source:** `GP25.stp` (SolidWorks STEP, one named solid per axis).
- **Pipeline (reproducible):** [`scripts/assets/`](../scripts/assets/) —
  FreeCAD tessellates each link → Blender welds vertices, decimates (collapse,
  no planar dissolve), assigns materials, nudges flush wrist parts to avoid
  z-fighting, exports `gp25.glb`.
- **Asset:** [`assets/robots/yaskawa_gp25/gp25.glb`](../assets/robots/yaskawa_gp25/)
  — Y-up, metres, articulated node chain `base→S→L→U→R→B→T→tcp`, materials
  *Yaskawa orange* (paint), *machined steel* (flange), *cast metal* (base).
  Stored via **Git LFS**.
- **Kinematics:** mirrored in C++ (`cavr::visualization::yaskawa_gp25()`,
  `robot_model.hpp`) and as portable
  [`gp25.kinematics.json`](../assets/robots/yaskawa_gp25/gp25.kinematics.json).
- Axis ranges/speeds from the official GP25 datasheet (S ±180°, L −105/+155°,
  U −86/+160°, R ±200°, B ±150°, T ±455°).

## Architecture (libraries)

Header-only `INTERFACE` libs, `cavr::*` namespaces, no third-party deps (JSON is
hand-rolled). Dependency flow is a clean DAG.

| Library | Role |
|---------|------|
| `libs/core` | `Vec3`, `Quaternion`, `Pose3D`, `Timestamp`/`Duration`. |
| `libs/machine` | **MachineProfile** (axes, frames, tool/user frames, IO, telemetry channels, cameras, motion vocabulary, weld defaults), `kinematics.hpp` (FK), **`ik.hpp`** (numerical inverse kinematics — damped least-squares over the FK Jacobian, works for any serial chain, respects joint limits), `json.hpp` + `profile_io.hpp` (import/export). |
| `libs/adapter_sdk` | **`RobotState`/TelemetryFrame** and the neutral **`ControllerAdapter`** interface (connect, discover_profile, load_task, start/pause/stop, `poll`, plus `move_to` for an immediate jog — the scene → robot direction). |
| `adapters/mock_robot` | **`MockController`** — deterministic GP25 cell that executes a precomputed joint trajectory and reports real telemetry. The reference adapter implementation. |
| `adapters/generic_tcp_robot` | **`GenericTcpController`** — a real `ControllerAdapter` over TCP, drop-in for the mock (`connect` a `host:port` instead of `"mock"`). Speaks a newline-delimited JSON protocol (`protocol.hpp`) to a controller bridge/PLC; server-pushed telemetry is drained non-blocking each `poll()`. Program control, live `move_to` jog and the **tool table** (`get_tools`/`select_tool`/`calibrate_tool`/`clear_tool`, mirrored client-side) all travel over the protocol, so frames + tools + Cartesian motion work with a remote robot. All platform socket code (Winsock/BSD) is confined to one TU (`tcp_connection.cpp`), which also provides a `TcpListener` for reference servers/tests. |
| `libs/validation` | **`trajectory_validator`** — joint-limit / speed / frame checks (collisions explicitly "not evaluated"). |
| `libs/runtime` | **Timeline** (`OperationStep`/`TimelineEvent`), **`SessionManager`** (Scan→Plan→Validate→Execute→Monitor→Replay), `SessionLog` + `session_io` (save/replay), `demo_plan`. Also bridges sessions onto the recording layer: `record_session` (write/read a whole `SessionLog`), `session_recorder` (live, incremental Monitor-phase sink), `camera_recording` (synchronized camera stream), `catalog_index` (recording → catalog row). |
| `libs/adapter_sdk` (camera) | `CameraFrame`/`CameraAdapter`; `adapters/mock_camera`'s **`MockCamera`** is the synthetic reference implementation, and `adapters/file_camera`'s **`FileCameraAdapter`** replays a real `.pgm`/`.ppm` image sequence from disk (dependency-free Netpbm reader/writer). `SessionManager::attach_camera` polls whichever is attached on the same tick as the robot, so recordings carry synchronized robot + vision. |
| `libs/record` | Storage-neutral recording model (`Channel`/`Message`, `RecordingWriter`/`RecordingReader`) plus the dependency-free JSON reference backend. `copy.hpp`'s `write_recording` replays a whole recording through any writer (remapping channel ids) — the backend-agnostic core of `cavr-convert`. |
| `libs/storage_mcap` | Authoritative **MCAP** backend (vendored foxglove/mcap, single TU, uncompressed) implementing the same interfaces, with a streaming (unchunked) mode for live recording. Gated by `CAVR_ENABLE_MCAP` (default `ON`); with it off the JSON backend is the only option and the tree stays dependency-free. |
| `libs/catalog` | Local session catalog — reconstructible metadata only (id, path, span, robot/camera model, file size/hash, tags, annotations, bookmarks, validation summaries); heavy data stays in the recording. Engine-neutral `Catalog` interface, `InMemoryCatalog` reference impl, `SqliteCatalog` (vendored amalgamation, PIMPL) gated by `CAVR_ENABLE_SQLITE` (default `ON`). |
| `libs/visualization` | `RobotModel` + FK + render-side scene data. |

## Backend CLIs

- **`cavr-record`** — runs the demo GP25 workflow against `MockController` (or,
  via `--tcp host:port`, a remote robot over the network) and either `MockCamera`
  or, via `--frames-dir`, a real `FileCameraAdapter` image sequence, streaming a
  live synchronized session to `--out` (`.mcap` or `.json`), and optionally
  indexes the finished recording into `--catalog` (SQLite when built, else
  in-memory). First end-to-end exercise of record → storage → catalog from the
  command line.
- **`cavr-inspect`** — dumps a recording's channels, message counts, session
  header and camera stream through the neutral `RecordingReader`; works on
  `.mcap` and `.json` alike.
- **`cavr-convert`** — converts a recording between backends (`.mcap` ↔ `.json`)
  by loading it into the neutral `record::Recording` and writing it back through
  the other backend; the message stream is preserved exactly and channel ids are
  remapped. Built on `record/copy.hpp`'s `write_recording`.
- **`cavr-validate`** — runs the pre-execution trajectory validation of the
  reference welding plan against a machine profile (built-in GP25, or one loaded
  from JSON), the same check the Studio Validate phase performs; exits non-zero
  on errors, linter-style.
- **`cavr-robotd`** — a **reference robot server**: speaks the `generic_tcp_robot`
  protocol over TCP, backed by the mock GP25, continuously looping the demo
  trajectory. Stands in for a per-vendor bridge so the whole channel (discover →
  load → live telemetry) runs end to end on any platform. `cavr-record --tcp
  host:port` records from it; `CAVR_ROBOT_ENDPOINT=host:port ./cavr-studio` makes
  the virtual GP25 mirror it live — the **robot → scene digital twin** (verified:
  the scene tracks the remote robot and freezes when it stops streaming).

## CAVR Studio (Qt 6 / Quick3D)

- Central viewport is **Qt Quick 3D** ([`RobotViewport.qml`](../apps/cavr-studio/qml/RobotViewport.qml))
  loading `gp25.glb` and driving the six joints.
- [`RobotController`](../apps/cavr-studio/src/RobotController.cpp) (QObject) runs
  the demo workflow through `SessionManager` + `MockController` and republishes
  **live telemetry** to QML: joint angles drive the robot, the overlay shows
  program state / current step / weld, and controller events stream into the
  Events panel and status bar. **No fake animation** — every frame is telemetry.
- Swapping `MockController` for a real `ControllerAdapter` changes nothing else
  — `adapters/generic_tcp_robot`'s `GenericTcpController` is exactly such a
  drop-in, connecting to a `host:port` over TCP instead of the in-process mock.
  Set **`CAVR_ROBOT_ENDPOINT=host:port`** to drive the scene from a remote robot
  (e.g. `cavr-robotd`) — the virtual GP25 then mirrors the real robot's live
  motion (robot → scene digital twin). Unset, it runs the standalone mock demo.
- The **Jog** panel commands the robot (mock or remote) live via
  `ControllerAdapter::move_to` — the **scene → robot** direction. It offers a
  **coordinate-system selector** (World/Base/Tool/User), a **speed field (mm/s)**,
  per-axis **joint jog** (±5°), full 6-axis **Cartesian jog** (X/Y/Z ±5 cm,
  Rx/Ry/Rz ±5°) expressed in the selected frame and solved through `ik.hpp`, and a
  **tool table** panel (select one of 10 slots, calibrate its TCP offset, clear).
  Jogging enters a manual hold; Run Demo resumes the cycle. With both directions
  wired, CAVR Studio is a bidirectional twin: it mirrors the robot's motion and
  commands it in the standard robot motion model (frames, tools, mm/s Cartesian).

## Build / CI / Releases

- One CMake tree builds on Linux/macOS/Windows. See [`CI_CD.md`](CI_CD.md).
- **CI** ([`ci.yml`](../.github/workflows/ci.yml)): build + `ctest` on 3 OSes,
  plus a Qt Studio build on 3 OSes.
- **Releases** ([`release.yml`](../.github/workflows/release.yml)): push a `v*`
  tag → per-OS bundled archives published to a GitHub Release.
- Tests (18, all green): `cavr_core_domain_types_test`, `cavr_replay_*`,
  `cavr_visualization_robot_model_test`, `cavr_runtime_workflow_test`
  (profile round-trip, validation, full session, save/replay),
  `cavr_record_recording_test`, `cavr_record_copy_test`,
  `cavr_storage_mcap_recording_test`, `cavr_runtime_session_recording_test`,
  `cavr_runtime_session_recorder_test`, `cavr_runtime_camera_recording_test`,
  `cavr_catalog_test`, `cavr_runtime_catalog_index_test`, `cavr_file_camera_test`, `cavr_machine_ik_test`, `cavr_machine_frames_test`,
  `cavr_generic_tcp_robot_test` (a fake robot server over loopback TCP drives the
  adapter and a full `SessionManager` session, plus a scene → robot `move_to` jog
  end to end and the mock's own live jog).

## What's next (natural extensions)

- A concrete controller bridge speaking the `generic_tcp_robot` protocol (or a
  vendor-SDK `ControllerAdapter`, e.g. `adapters/robodk`, still an empty
  placeholder). `GenericTcpController` is done and validated against a fake
  server; what remains is a real robot/PLC on the other end.
- A real image-decoding `CameraAdapter` (`adapters/opencv_camera` — currently
  an empty placeholder) or a live capture device. `adapters/file_camera`'s
  `FileCameraAdapter` already replays a real (Netpbm) image sequence from disk
  and is wired into `cavr-record --frames-dir`, but that is dependency-free
  disk replay, not decoding PNG/JPEG or a live camera.
- Camera/point-cloud ingestion + hand-eye into the scan step.
- Bind the remaining Studio panels (Telemetry, Calibration) to the data model.
- Interactive timeline editing + replay scrubbing from a saved `SessionLog`.
