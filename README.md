# CAVR

CAVR: Calibration-Aware Validation and Replay for Vision-Guided Industrial Robotics is planned as a ROS-free C++20 framework for recording, replaying, visualizing, and validating synchronized industrial robot and camera data.

## Naming

- Project name: CAVR.
- Full name: CAVR: Calibration-Aware Validation and Replay for Vision-Guided Industrial Robotics.
- Application name: CAVR Studio.
- Research benchmark name: CAVR-Bench.

This repository currently contains the project scaffold and an early Qt 6 Widgets CAVR Studio shell. Runtime robotics replay, recording, validation, and storage functionality are not implemented yet.


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

## Current Scope

- C++20 project foundation.
- CMake and Ninja presets for macOS development.
- Conan 2 recipe skeleton.
- Modular directory layout for core libraries, adapters, command-line apps, tests, datasets, scripts, and documentation.
- Qt 6 Widgets CAVR Studio shell with dock panels and a 3D viewport.

## Non-Scope

- No robot or camera SDK integration.
- No MCAP or Protocol Buffers implementation.
- No production backend-connected desktop application.
- No ROS dependency.
- No executable replay logic yet.
- CAVR Studio currently provides only a Qt-based UI shell.

## First Local Configure

```bash
cmake --preset mac-debug
cmake --build --preset mac-debug
ctest --preset mac-debug
```
