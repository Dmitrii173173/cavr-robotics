# Machine

Controller-neutral **machine configuration** layer. A `MachineProfile` is the
persistent description of one physical robot/machine: model, axes/joints,
coordinate frames, tool/user frames, IO, telemetry channels, cameras (incl.
depth / point-cloud / hand-eye), supported motion primitives and welding
process defaults.

Profiles are produced by connecting to a controller and discovering its
variables (see `cavr::adapter_sdk::ControllerAdapter`), then **saved as JSON**
so the same machine can be reconnected and reused without manual setup. Import
and export are dependency-free (`profile_io.hpp` on top of the small
`cavr::json` value type).

```cpp
auto result = cavr::machine::load_profile("profiles/gp25_cell1.json");
if (result.ok()) {
  const cavr::machine::MachineProfile& profile = result.profile;
}
```

This module holds **data and vocabulary only** — no rendering, no I/O transport.
Adapters map vendor-specific channels onto these structures; the runtime layer
drives sessions from them.
