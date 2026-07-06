# libs/sim

`VirtualRobot` — the stateful simulation of one robot, with no Qt, no sockets and
no notion of a connection. It owns the joint state, tool table and IO map, executes
a `libs/motion` `Trajectory` against wall-clock time, drives process signals (the
weld arc) and emits controller events, and reports everything as an
`adapter_sdk::RobotState`, exactly as a real controller's telemetry would.

This is where robot behaviour lives, separately from "how the app talks to the
controller". `adapters/mock_robot`'s `MockController` is a thin `ControllerAdapter`
over a `VirtualRobot`; the same backend runs inside `cavr-robotd`. Because it is
adapter-free, motion can be driven and asserted in a unit test without sockets, and
future realism (collisions, dynamics, fault models) has a home here instead of
bloating the mock.

The trajectory it executes is built by `motion::plan_task` — the same planner the
validator uses for cycle time and reachability — so what is checked and what runs
are one code path.
