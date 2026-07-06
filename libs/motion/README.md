# libs/motion

Time-parameterised motion planning, extracted from the mock controller so that
trajectory generation can be tested in isolation and shared between execution and
validation.

Layers, bottom to top:

- **`profile.hpp`** — `VelocityProfile`, a scalar speed profile over a 1-D path
  coordinate. `plan_trapezoidal` (bounded acceleration, optional non-zero boundary
  speeds) and `plan_s_curve` (bounded jerk, rest-to-rest). Pure and unit-agnostic;
  `sample(t)` returns `{s, v, a}`. Guarantees: `s` monotonic, `|v| <= v_max`,
  `|a| <= a_max`, `s(0)=0`, `s(duration)=length`.
- **`trajectory.hpp`** — `Trajectory`, a sequence of joint-space `PlanSegment`s laid
  on one clock, each with a velocity profile. `plan(segments, limits)` builds it and
  applies **corner blending**: a segment with a `blend_radius` does not stop at its
  end — the boundary carries a matched speed into the next segment and the joint
  path near the shared knot is rounded by a smooth cross-fade.
- **`plan_task.hpp`** — `plan_task(...)`, which turns an industrial `MotionTask`
  (MoveJ/MoveL/MoveC/Wait/Tool…) into a `Trajectory` using libs/machine FK/IK/arc.
  This is the one planner both `VirtualRobot` (execution) and `libs/validation`
  (cycle time / reachability) call — one source of truth for "what will run".

No Qt, no sockets, no controller I/O — that lives above, in `libs/sim`.
