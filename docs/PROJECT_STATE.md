# CAVR — Project State

A shared snapshot of where CAVR Studio is, so everyone has the same picture.
Keep this updated as the architecture evolves.

_Last updated: 2026-06-30._

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
| `libs/machine` | **MachineProfile** (axes, frames, tool/user frames, IO, telemetry channels, cameras, motion vocabulary, weld defaults), `kinematics.hpp` (FK), `json.hpp` + `profile_io.hpp` (import/export). |
| `libs/adapter_sdk` | **`RobotState`/TelemetryFrame** and the neutral **`ControllerAdapter`** interface (connect, discover_profile, load_task, start/pause/stop, `poll`). |
| `adapters/mock_robot` | **`MockController`** — deterministic GP25 cell that executes a precomputed joint trajectory and reports real telemetry. The reference adapter implementation. |
| `libs/validation` | **`trajectory_validator`** — joint-limit / speed / frame checks (collisions explicitly "not evaluated"). |
| `libs/runtime` | **Timeline** (`OperationStep`/`TimelineEvent`), **`SessionManager`** (Scan→Plan→Validate→Execute→Monitor→Replay), `SessionLog` + `session_io` (save/replay), `demo_plan`. |
| `libs/visualization` | `RobotModel` + FK + render-side scene data. |

## CAVR Studio (Qt 6 / Quick3D)

- Central viewport is **Qt Quick 3D** ([`RobotViewport.qml`](../apps/cavr-studio/qml/RobotViewport.qml))
  loading `gp25.glb` and driving the six joints.
- [`RobotController`](../apps/cavr-studio/src/RobotController.cpp) (QObject) runs
  the demo workflow through `SessionManager` + `MockController` and republishes
  **live telemetry** to QML: joint angles drive the robot, the overlay shows
  program state / current step / weld, and controller events stream into the
  Events panel and status bar. **No fake animation** — every frame is telemetry.
- Swapping `MockController` for a real `ControllerAdapter` (e.g. in
  `adapters/generic_tcp_robot`) changes nothing else.

## Build / CI / Releases

- One CMake tree builds on Linux/macOS/Windows. See [`CI_CD.md`](CI_CD.md).
- **CI** ([`ci.yml`](../.github/workflows/ci.yml)): build + `ctest` on 3 OSes,
  plus a Qt Studio build on 3 OSes.
- **Releases** ([`release.yml`](../.github/workflows/release.yml)): push a `v*`
  tag → per-OS bundled archives published to a GitHub Release.
- Tests: `cavr_core_domain_types_test`, `cavr_replay_*`,
  `cavr_visualization_robot_model_test`, `cavr_runtime_workflow_test`
  (profile round-trip, validation, full session, save/replay).

## What's next (natural extensions)

- A real `ControllerAdapter` (TCP / vendor SDK) replacing the mock.
- Camera/point-cloud ingestion + hand-eye into the scan step.
- Bind the remaining Studio panels (Telemetry, Calibration) to the data model.
- Interactive timeline editing + replay scrubbing from a saved `SessionLog`.
- IK for Cartesian (MoveL/MoveC) targets in planning/validation.
