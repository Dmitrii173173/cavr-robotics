# CAVR — Project State

A shared snapshot of where CAVR Studio is, so everyone has the same picture.
Keep this updated as the architecture evolves.

_Last updated: 2026-07-09._

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
| `libs/machine` | **MachineProfile** (axes, frames, tool/user frames, IO, telemetry channels, cameras, motion vocabulary, weld defaults), `kinematics.hpp` (FK), **`ik.hpp`** (numerical inverse kinematics — damped least-squares over the FK Jacobian, works for any serial chain, respects joint limits), `arc.hpp` (circular-arc interpolation for MoveC), `json.hpp` + `profile_io.hpp` (import/export). |
| `libs/motion` | Time-parameterised motion planning, extracted so it can be tested in isolation and shared by execution **and** validation. `profile.hpp` (`VelocityProfile` — trapezoidal / bounded-jerk S-curve `sample(t)→{s,v,a}`), `trajectory.hpp` (joint-space `Trajectory` with **corner blending** — a segment with a `blend_radius` carries speed through the shared knot instead of stopping), `plan_task.hpp` (`plan_task` turns a `MotionTask` into a `Trajectory` via machine FK/IK/arc — the one planner both execution and validation call). No Qt / sockets / controller I/O. |
| `libs/sim` | **`VirtualRobot`** — the stateful, adapter-free simulation of one robot: owns joint state, tool table and IO map, executes a `libs/motion` `Trajectory` against a supplied clock, drives the weld process signals and emits controller events, and reports everything as an `adapter_sdk::RobotState` exactly as a real controller's telemetry would. This is where robot behaviour lives (and where future realism — collisions, dynamics, faults — belongs), separate from "how the app talks to the controller". |
| `libs/adapter_sdk` | **`RobotState`/TelemetryFrame** and the neutral **`ControllerAdapter`** interface (connect, discover_profile, load_task, start/pause/stop, `poll`, plus `move_to` for an immediate jog — the scene → robot direction). |
| `adapters/mock_robot` | **`MockController`** — a thin `ControllerAdapter` over a `cavr::sim::VirtualRobot`: the adapter owns only the connection concern and forwards behaviour to the `VirtualRobot`. The same backend runs inside `cavr-robotd`. The reference adapter implementation; ships several built-in profiles (GP25 welding cell, a vendor-neutral PNR robot). |
| `adapters/generic_tcp_robot` | **`GenericTcpController`** — a real `ControllerAdapter` over TCP, drop-in for the mock (`connect` a `host:port` instead of `"mock"`). Speaks a newline-delimited JSON protocol (`protocol.hpp`) to a controller bridge/PLC; server-pushed telemetry is drained non-blocking each `poll()`. Program control, live `move_to` jog and the **tool table** (`get_tools`/`select_tool`/`calibrate_tool`/`clear_tool`, mirrored client-side) all travel over the protocol, so frames + tools + Cartesian motion work with a remote robot. All platform socket code (Winsock/BSD) is confined to one TU (`tcp_connection.cpp`), which also provides a `TcpListener` for reference servers/tests. |
| `libs/validation` | **`trajectory_validator`** — joint-limit / speed / frame / reachability / cycle-time checks, plus **collision checking** (`collision.hpp`): the robot is approximated as link capsules and the planned trajectory is sampled and checked, at each configuration, for self-collision (non-adjacent links), floor penetration (a plane on a chosen up-axis), and sphere obstacles — all exact segment/segment and segment/sphere/plane distances. `validate_task(profile, task, CollisionModel)` sets `collisions_evaluated = true` and reports the worst penetration per pair; boxes/meshes are deliberately not modelled yet rather than approximated. |
| `libs/runtime` | **Timeline** (`OperationStep`/`TimelineEvent`), **`SessionManager`** (Scan→Plan→Validate→Execute→Monitor→Replay), `SessionLog` + `session_io` (save/replay), `demo_plan`. Bridges sessions onto the recording layer: `record_session` (write/read a whole `SessionLog`), `session_recorder` (live Monitor-phase sink), `camera_recording` + `point_cloud_recording` (synchronized image + 3D streams), `catalog_index` (recording → catalog row). **`vision_guidance`** closes the Vision-Guided loop: `cloud_in_base` places a scan `PointCloud` in the base frame via the hand-eye calibration + flange pose, `seam_offset` compares the observed seam to the planned one, and `apply_seam_offset` shifts the Cartesian targets of a `MotionTask` by that correction. |
| `libs/adapter_sdk` (camera) | `CameraFrame`/`CameraAdapter` plus **`PointCloud`** (3D geometry: points + optional per-point colors/normals, time-stamped in a named sensor frame). `CameraAdapter::poll_point_cloud` is an optional depth/scan output (defaults to none, so 2D-only adapters are unaffected); `adapters/mock_camera`'s **`MockCamera`** is the synthetic reference and now emits both a frame and a synthetic scan cloud, and `adapters/file_camera`'s **`FileCameraAdapter`** replays a real `.pgm`/`.ppm`/`.png` image sequence from disk (dependency-free Netpbm reader/writer plus a **from-scratch PNG decoder** — a complete DEFLATE inflater and PNG unfilter, no libpng/zlib — decoding grayscale/truecolor/indexed/alpha 8-bit images). `SessionManager::attach_camera` polls whichever is attached on the same tick as the robot, streaming a synchronized robot + image + point-cloud session (`runtime::point_cloud_recording` serializes the cloud channel, mirroring `camera_recording`). |
| `libs/record` | Storage-neutral recording model (`Channel`/`Message`, `RecordingWriter`/`RecordingReader`) plus the dependency-free JSON reference backend. `copy.hpp`'s `write_recording` replays a whole recording through any writer (remapping channel ids) — the backend-agnostic core of `cavr-convert`. |
| `libs/storage_mcap` | Authoritative **MCAP** backend (vendored foxglove/mcap, single TU, uncompressed) implementing the same interfaces, with a streaming (unchunked) mode for live recording. Gated by `CAVR_ENABLE_MCAP` (default `ON`); with it off the JSON backend is the only option and the tree stays dependency-free. |
| `libs/catalog` | Local session catalog — reconstructible metadata only (id, path, span, robot/camera model, file size/hash, tags, annotations, bookmarks, validation summaries); heavy data stays in the recording. Engine-neutral `Catalog` interface, `InMemoryCatalog` reference impl, `SqliteCatalog` (vendored amalgamation, PIMPL) gated by `CAVR_ENABLE_SQLITE` (default `ON`). |
| `libs/visualization` | `RobotModel` + FK + render-side scene data. |
| `libs/calibration` | The **Calibration-Aware** data model + estimation: `CameraIntrinsics` (pinhole + Brown-Conrady distortion) with `project`/`unproject`/`reprojection_error`; `HandEyeCalibration` (camera↔flange / camera↔base SE(3) with method / residual / uncertainty / version metadata) plus `camera_in_base` and `point_base_to_camera` frame algebra; and `solve_hand_eye` — a dependency-free **Tsai-Lenz** estimator that recovers the hand-eye transform (eye-in-hand or eye-to-hand) from synchronized (robot pose, target-in-camera) samples via AX = XB, hand-rolled 3x3 linear algebra, reporting the RMS residual. JSON import/export throughout. Intrinsics *fitting* and wiring vision into the plan are the remaining pieces. |

Reserved (README-only scaffolds, no code yet — not in the first MVP):
`libs/fault_injection` (deterministic delay/drop/noise scenarios),
`libs/frame_graph` (timestamped SE(3) transform tree), `libs/transport`
(async TCP/UDP/serial, reconnect/heartbeat), `libs/time` (session clocks,
source-clock mapping, deterministic scheduling).

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
- The **Program** panel shows live collision status (self + floor) alongside
  validation; the **Calibration** panel shows live camera intrinsics, runs a
  **hand-eye teach + Tsai-Lenz solve** (Capture poses → Solve → save intrinsics +
  hand-eye JSON, with the recovered transform and RMS residual shown), and the
  live vision-guided seam offset with an Apply button; the **Session / Replay**
  panel loads a recorded `SessionLog` and scrubs it through the scene (open,
  timeline slider, play/pause, exit) — the same viewport driven from a file
  instead of the live robot.

## Build / CI / Releases

- One CMake tree builds on Linux/macOS/Windows. See [`CI_CD.md`](CI_CD.md).
- **CI** ([`ci.yml`](../.github/workflows/ci.yml)): build + `ctest` on 3 OSes,
  plus a Qt Studio build on 3 OSes.
- **Releases** ([`release.yml`](../.github/workflows/release.yml)): push a `v*`
  tag → per-OS bundled archives published to a GitHub Release.
- Tests (30, all green): `cavr_core_domain_types_test`, `cavr_replay_*`,
  `cavr_visualization_robot_model_test`, `cavr_runtime_workflow_test`
  (profile round-trip, validation, full session, save/replay),
  `cavr_record_recording_test`, `cavr_record_copy_test`,
  `cavr_storage_mcap_recording_test`, `cavr_runtime_session_recording_test`,
  `cavr_runtime_session_recorder_test`, `cavr_runtime_camera_recording_test`,
  `cavr_catalog_test`, `cavr_runtime_catalog_index_test`, `cavr_file_camera_test`,
  `cavr_machine_ik_test`, `cavr_machine_frames_test`, `cavr_machine_arc_test`,
  `cavr_profile_store_test`, `cavr_program_store_test`,
  `cavr_motion_profile_test` (trapezoidal / S-curve velocity profiles),
  `cavr_motion_trajectory_test` (corner blending vs. stop-at-knot),
  `cavr_sim_virtual_robot_test` (VirtualRobot lifecycle: idle → run → complete,
  pause/resume freeze, jog reachability, IO direction rules),
  `cavr_calibration_test` (pinhole+distortion project/unproject round-trip,
  reprojection error, hand-eye eye-in-hand / eye-to-hand frame algebra, JSON
  round-trip),
  `cavr_hand_eye_solver_test` (Tsai-Lenz recovers a known transform from
  synthetic closed-loop samples, both mounting styles),
  `cavr_runtime_vision_guidance_test` (scan cloud → base frame, seam offset, and
  Cartesian plan correction, end to end),
  `cavr_validation_collision_test` (clean cell passes, floor / obstacle-sphere /
  self-collision each detected over the sampled trajectory),
  `cavr_runtime_point_cloud_recording_test` (synchronized robot + scan cloud
  streamed and read back verbatim, JSON and MCAP),
  `cavr_file_camera_png_test` (decodes a real dynamic-Huffman-compressed PNG,
  stored-block writer round-trip, adapter replays a `.png`),
  `cavr_generic_tcp_robot_test` (a fake robot server over loopback TCP drives the
  adapter and a full `SessionManager` session, plus a scene → robot `move_to` jog
  end to end and the mock's own live jog).

## Vision-Guided pipeline (done)

The calibration + vision workstream is complete and dependency-free, end to end:

- **`PointCloud`** 3D data flows through the synchronized session and recording
  (`adapter_sdk` + `runtime/point_cloud_recording`); `MockCamera` emits a scan
  cloud each tick.
- **From-scratch PNG decoder** (`adapters/file_camera/png.hpp`) replays real
  `.png` frames — a full DEFLATE inflater + unfilter, no libpng/zlib.
- **Calibration model** (`libs/calibration`): pinhole + Brown-Conrady intrinsics
  (`project`/`unproject`/reprojection) and `HandEyeCalibration`, with a
  **Tsai-Lenz** `solve_hand_eye` estimating the transform from captured samples.
- **Vision guidance** (`runtime/vision_guidance`): scan cloud → base frame →
  seam offset → Cartesian plan correction.

What remains to make it production vision: a real image-decoding / live-capture
`CameraAdapter` emitting true depth clouds (`adapters/opencv_camera` is still a
placeholder), camera-intrinsics *fitting* (checkerboard), and a real feature
detector behind `seam_offset` (today a centroid).

## What's next (natural extensions)

- Richer collision geometry: box/mesh obstacles and per-link radii (today the
  model is link capsules vs self / floor / sphere obstacles), and surfacing the
  `CollisionModel` in the Studio Validate phase.
- A concrete controller bridge speaking the `generic_tcp_robot` protocol (or a
  vendor-SDK `ControllerAdapter`, e.g. `adapters/robodk`, still an empty
  placeholder). `GenericTcpController` is done and validated against a fake
  server; what remains is a real robot/PLC on the other end.
- Bind the remaining Studio panel (Telemetry) to real data; camera-intrinsics
  *fitting* (checkerboard) to refine the representative intrinsics the UI shows.
- Interactive timeline *editing* (the Session/Replay panel already scrubs a saved
  `SessionLog` through the scene — load, seek, play/pause).
