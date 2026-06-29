<img width="1536" height="1024" alt="cavr_preview" src="https://github.com/user-attachments/assets/44b8ad20-3a0d-4745-8f04-188ce2d143d8" />



# CAVR

[![CI](https://github.com/Dmitrii173173/cavr-robotics/actions/workflows/ci.yml/badge.svg)](https://github.com/Dmitrii173173/cavr-robotics/actions/workflows/ci.yml)

CAVR: Calibration-Aware Validation and Replay for Vision-Guided Industrial Robotics is planned as a ROS-free C++20 framework for recording, replaying, visualizing, and validating synchronized industrial robot and camera data.

> **Build, CI & releases:** [docs/CI_CD.md](docs/CI_CD.md) · **Architecture & current state:** [docs/PROJECT_STATE.md](docs/PROJECT_STATE.md)
>
> Builds on Linux, macOS and Windows via GitHub Actions; pushing a `v*` tag publishes per-platform release archives automatically.

## Naming

- Project name: CAVR.
- Full name: CAVR: Calibration-Aware Validation and Replay for Vision-Guided Industrial Robotics.
- Application name: CAVR Studio.
- Research benchmark name: CAVR-Bench.

This repository currently contains the project scaffold and an early Qt 6 Widgets CAVR Studio shell. Runtime robotics replay, recording, validation, and storage functionality are not implemented yet.



## Run CSV Replay Demo

```bash
cmake --preset mac-debug
cmake --build --preset mac-debug
./build/mac-debug/apps/cavr-play/cavr-play datasets/demo_csv/session.json
```

Expected output:

```text
0.000 s | pose 0 | frame_0000.png
0.100 s | pose 1 | frame_0001.png
0.200 s | pose 2 | frame_0002.png
```

## Run CAVR Studio UI

```bash
cmake --preset mac-debug
cmake --build --preset mac-debug
./build/mac-debug/apps/cavr-studio/cavr-studio
```

The shell also supports a non-GUI smoke check:

```bash
./build/mac-debug/apps/cavr-studio/cavr-studio --help
```

The central viewport is a live **Qt Quick 3D** scene that loads the articulated
[Yaskawa GP25](assets/robots/yaskawa_gp25/README.md) (`gp25.glb`) and drives its
six joints. Building CAVR Studio therefore needs the Qt 6 modules
**Widgets, OpenGLWidgets, Quick, Quick3D, Qml, QuickWidgets** (point CMake at the
Qt install via `CMAKE_PREFIX_PATH`). When Qt 6 is absent the app target is
skipped and the rest of the project still builds.

## Robot Assets

Articulated 3D robot models live under [`assets/robots/`](assets/README.md). The
first is the **Yaskawa Motoman GP25** (6-axis), built from CAD into a `glTF` asset
whose joints are named nodes placed on their real rotation axes, so the model can
be posed and made to replay recorded robot motion. Its kinematics (axes, origins,
datasheet limits, forward kinematics) are available in C++ via
`cavr::visualization::yaskawa_gp25()`
([`robot_model.hpp`](libs/visualization/include/cavr/visualization/robot_model.hpp))
and as a portable
[`gp25.kinematics.json`](assets/robots/yaskawa_gp25/gp25.kinematics.json)
descriptor. Assets are reproducible from CAD with
[`scripts/assets/`](scripts/assets/README.md).

## Current Scope

- C++20 project foundation.
- CMake and Ninja presets for macOS development.
- Conan 2 recipe skeleton.
- Modular directory layout for core libraries, adapters, command-line apps, tests, datasets, scripts, and documentation.
- Qt 6 Widgets CAVR Studio shell with dock panels and a 3D viewport.
- Deterministic CSV pose and image metadata replay demo.
- Articulated Yaskawa GP25 robot asset with C++ forward kinematics.

## Non-Scope

- No robot or camera SDK integration.
- No MCAP or Protocol Buffers implementation.
- No production backend-connected desktop application.
- No ROS dependency.
- No MCAP-backed replay logic yet.
- CAVR Studio currently provides only a Qt-based UI shell.

## First Local Configure

```bash
cmake --preset mac-debug
cmake --build --preset mac-debug
ctest --preset mac-debug
```
