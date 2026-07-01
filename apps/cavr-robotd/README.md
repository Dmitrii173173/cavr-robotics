# cavr-robotd

A **reference robot server** that speaks the
[`generic_tcp_robot`](../../adapters/generic_tcp_robot/) wire protocol over TCP,
backed by the deterministic mock GP25. It lets a CAVR client (Studio,
`cavr-record`) drive and monitor a robot across the network without any vendor
SDK on the client side.

```bash
cavr-robotd --port 9010
# then, in another shell / on another machine:
cavr-record --tcp 127.0.0.1:9010 --out remote.mcap
# or point CAVR Studio at it:
CAVR_ROBOT_ENDPOINT=127.0.0.1:9010 ./cavr-studio
```

It stands in for the **per-vendor bridge** in the universal-adapter architecture:
a real bridge runs next to the robot and translates this same protocol to a
vendor controller SDK (e.g. a PNR/Crp control card), while CAVR itself stays
vendor-free and cross-platform. `cavr-robotd` uses `MockController` instead, so
the whole channel — profile discovery, task load, live telemetry — is exercised
end to end on any platform.

While a client is executing it continuously loops the demo welding trajectory, so
the robot keeps moving: connect CAVR Studio and the virtual GP25 mirrors this
"remote robot" live — the **robot → scene** digital-twin direction.

- `--port <n>` — TCP port to listen on (default 9010).
- `--rate-hz <n>` — telemetry frames per second while running (default 50).
