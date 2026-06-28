# CAVR

CAVR: Calibration-Aware Validation and Replay for Vision-Guided Industrial Robotics is planned as a ROS-free C++20 framework for recording, replaying, visualizing, and validating synchronized industrial robot and camera data.

## Naming

- Project name: CAVR.
- Full name: CAVR: Calibration-Aware Validation and Replay for Vision-Guided Industrial Robotics.
- Application name: CAVR Studio.
- Research benchmark name: CAVR-Bench.

This repository currently contains only the project scaffold. No runtime implementation is present yet.

## Current Scope

- C++20 project foundation.
- CMake and Ninja presets for macOS development.
- Conan 2 recipe skeleton.
- Modular directory layout for core libraries, adapters, command-line apps, tests, datasets, scripts, and documentation.

## Non-Scope

- No robot or camera SDK integration.
- No MCAP or Protocol Buffers implementation.
- No GUI.
- No ROS dependency.
- No executable replay logic yet.

## First Local Configure

```bash
cmake --preset mac-debug
cmake --build --preset mac-debug
ctest --preset mac-debug
```
