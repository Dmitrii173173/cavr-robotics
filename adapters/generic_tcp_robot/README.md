# Generic TCP Robot Adapter

A real `ControllerAdapter` that speaks a small newline-delimited JSON protocol to
a controller bridge / PLC over TCP. It is a **drop-in replacement for
`MockController`**: point `SessionManager::connect` at a `host:port` endpoint
instead of `"mock"` and nothing else in the runtime or CAVR Studio changes.

```cpp
generic_tcp_robot::GenericTcpController controller;
manager.connect(controller, {"192.168.1.50:9010", "tcp"});
manager.discover_profile();          // pulls the MachineProfile from the controller
manager.set_plan(plan);
manager.validate();
manager.execute("session_0");
manager.tick(now);                   // poll() drains the pushed telemetry stream
```

## Protocol

One JSON object per line. The adapter is the **client**; the controller is the
**server**. See [`protocol.hpp`](include/cavr/adapters/generic_tcp_robot/protocol.hpp)
for the canonical serializers (they reuse the project's profile/pose JSON).

**Client → server (commands)**

```
{"cmd":"discover_profile"}
{"cmd":"load_task","task":[<command>, ...]}
{"cmd":"start"} | {"cmd":"pause"} | {"cmd":"resume"} | {"cmd":"stop"}
```

**Server → client (replies + telemetry)**

```
{"type":"profile","profile":{<MachineProfile>}}   # reply to discover_profile
{"type":"ack","cmd":"start","ok":true}            # reply to a command
{"type":"state", <RobotState fields>}             # unsolicited telemetry stream
```

Telemetry is server-pushed. `poll()` is non-blocking: it drains whatever state
frames arrived since the last tick and returns the latest, stamped with the
caller's clock so it stays aligned with the session clock and any synchronized
camera. Telemetry that arrives while the adapter is awaiting a command ack is
buffered and delivered on the next `poll()`, never dropped.

## Structure

- [`tcp_connection.hpp`](include/cavr/adapters/generic_tcp_robot/tcp_connection.hpp)
  / [`src/tcp_connection.cpp`](src/tcp_connection.cpp) — cross-platform TCP client
  (`TcpConnection`) and listener (`TcpListener`). All platform socket code
  (Winsock / BSD sockets) is confined to the one `.cpp`; the header exposes no
  platform types, so this is the module's only compiled translation unit.
- `protocol.hpp` — the wire vocabulary (header-only).
- [`generic_tcp_controller.hpp`](include/cavr/adapters/generic_tcp_robot/generic_tcp_controller.hpp)
  — `GenericTcpController`, the `ControllerAdapter` implementation (header-only).

`TcpListener` exists so a reference server or a test can accept a connection and
speak the protocol without duplicating socket code; the adapter itself never
listens.
