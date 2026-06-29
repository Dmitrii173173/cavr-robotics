# Build, CI & Releases

CAVR builds on **Linux, macOS and Windows** from one CMake tree. Continuous
integration and releases are automated with GitHub Actions.

## Continuous integration — [`.github/workflows/ci.yml`](../.github/workflows/ci.yml)

Runs on every push and pull request to `main`.

| Job | What it does |
|-----|--------------|
| **build & test** | Configures with Ninja, builds the core libraries, adapters and CLI apps, and runs the full `ctest` suite on `ubuntu-latest`, `macos-latest`, `windows-latest`. Qt is *not* installed — the `cavr-studio` GUI target detects the missing Qt and skips itself, keeping this job fast and reliable. |
| **build studio** | Installs Qt 6.8 (with `qtquick3d qtshadertools qtquicktimeline qt5compat`) and builds the `cavr-studio` Quick3D desktop app on all three platforms, so the GUI keeps compiling. |

Toolchain: CMake + Ninja via [`lukka/get-cmake`](https://github.com/lukka/get-cmake),
MSVC via [`ilammy/msvc-dev-cmd`](https://github.com/ilammy/msvc-dev-cmd), Qt via
[`jurplel/install-qt-action`](https://github.com/jurplel/install-qt-action).

## Releases — [`.github/workflows/release.yml`](../.github/workflows/release.yml)

Releases are **fully automatic**. To cut one, push a tag:

```bash
git tag v0.1.0
git push origin v0.1.0
```

The workflow then, on each OS:

1. installs Qt 6.8 and builds CAVR Studio + the CLI in Release;
2. bundles the Qt runtime — `windeployqt` (Windows), `macdeployqt` on the
   `.app` (macOS), a launcher script + `assets/` (Linux);
3. packages an archive per platform:
   - `cavr-studio-windows-x64.zip`
   - `cavr-studio-macos.zip`
   - `cavr-studio-linux-x64.tar.gz`
4. publishes a GitHub Release for the tag with all archives attached and
   auto-generated notes.

Version numbers come from `project(CAVR VERSION ...)` in the root
[`CMakeLists.txt`](../CMakeLists.txt); keep the tag in sync.

## Local builds

### Linux / macOS

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

### Windows (MSVC)

From a *Developer* shell (or after `vcvars64.bat`):

```bat
cmake -S . -B build -G "Ninja" -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

### Building CAVR Studio (Qt)

The GUI needs Qt 6 with **Widgets, OpenGLWidgets, Quick, Quick3D, Qml,
QuickWidgets** plus the `qtquicktimeline` module (required by Quick3D's runtime
asset loader). Point CMake at the Qt install:

```bash
cmake -S . -B build -G Ninja -DCMAKE_PREFIX_PATH=/path/to/Qt/6.8.3/<arch>
cmake --build build --target cavr-studio
```

When Qt is not found the app target is skipped and the rest of the project still
builds.

## Large assets (Git LFS)

The robot mesh `assets/robots/yaskawa_gp25/gp25.glb` (~50 MB) is stored with
**Git LFS** (see [`.gitattributes`](../.gitattributes)). Clone with LFS enabled:

```bash
git lfs install
git clone <repo>
```

CI checks out with `lfs: true` automatically.
