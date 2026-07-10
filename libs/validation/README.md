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
- **`collision.hpp`** — the collision model. The robot is approximated as a chain
  of **link capsules** (a segment between consecutive joint origins, swept by a
  radius, plus the tool stub). `check_configuration(axes, q, tool, model)` checks
  one joint configuration for:
  - **self-collision** — non-adjacent link capsules against each other,
  - **floor** — a plane on a chosen up-axis (the GP25 asset is Y-up), with optional
    clearance,
  - **sphere obstacles** — fixtures/posts as bounding spheres.

  All distances are exact (segment/segment, segment/plane, segment/point); boxes
  and meshes are intentionally not modelled yet rather than approximated.

The `validate_task(profile, task, CollisionModel)` overload runs the base checks,
then **samples the planned trajectory** and checks every configuration, reporting
the worst penetration per colliding pair and setting `collisions_evaluated = true`.
