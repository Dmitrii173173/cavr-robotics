# Fault Injection

Deterministic fault models for the virtual robot. This is the library that gives a
simulated controller the failure behaviour a real one has — so an operator can
rehearse an alarm, and a test can assert the robot handles it — without any Qt,
sockets, or clock of its own. A `cavr::sim::VirtualRobot` owns one `FaultInjector`
and applies its verdict on each `poll(now)` tick.

## Model

- **`DeterministicRng`** — a SplitMix64 stream. Fully specified by its seed, so a
  scenario replays bit-for-bit on every platform. Faults are reproducible, never
  flaky.
- **`FaultSpec`** — one fault, with a `FaultKind` and a `TriggerKind`:
  - Kinds: `EmergencyStop`, `ServoFault` (both abort the run and fault the servos),
    `ArcLoss` (drops the weld signals without aborting), `EncoderNoise` and
    `CalibrationDrift` (continuous telemetry perturbations).
  - Triggers: `AtTime` / `AtStep` (one-shot), `Rate` (recurring, seeded), `Always`
    (continuous effects).
- **`FaultInjector`** — holds the armed scenario. `poll(elapsed, dt, step, prev_step)`
  returns the discrete faults that tripped this tick; `perturb_joints()` and
  `tcp_drift()` apply the continuous ones. `arm()` rewinds it to the seed (the
  operator's "reset").

Triggers and continuous noise draw from two independent RNG streams, so adding noise
to a scenario does not shift when its timed/rate faults fire.

## Example

```cpp
robot.faults().add(cavr::fault_injection::estop_at(3.0));        // E-stop 3 s in
robot.faults().add(cavr::fault_injection::encoder_noise(0.001)); // 0.001 rad jitter
robot.start();  // arms the scenario; poll() drives it deterministically
```

Covered by `tests/fault_injection_test.cpp` (RNG, triggers, noise, drift) and the
fault cases in `tests/sim_virtual_robot_test.cpp` (E-stop aborts, arc loss drops the
weld).
