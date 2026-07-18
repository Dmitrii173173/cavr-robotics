# libs/validation

Pre-execution checks on a planned `MotionTask` against a `MachineProfile` — the
Validate phase of a session, run before anything is sent to the controller.
Dependency-free.

- **`trajectory_validator.hpp`** — `validate_task(profile, task)` returns a
  `ValidationReport` (a list of severity-tagged `Issue`s + estimated cycle time).
  It checks joint counts, joint-limit and speed compliance, referenced tool/user
  frames, weld-capability, and — from the **same `libs/motion` planner the
  controller runs** — reachability (a Cartesian target whose IK leaves the
  workspace is a hard error) and cycle time. So "what was validated" and "what
  will run" are one code path.

Full collision and singularity analysis is intentionally out of scope: the report
carries a `collisions_evaluated` flag that stays `false`, so downstream consumers
know these were not checked rather than assuming a silent pass. The robot's own
constraints — joint range, speed and reachability — are what the validator
enforces.
