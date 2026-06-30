# Mock Camera Adapter

`MockCamera` — the reference `cavr::adapter_sdk::CameraAdapter` implementation,
mirroring mock_robot's `MockController`. Each `poll()` emits a small, deterministic
grayscale (`mono8`) frame whose pattern varies with the timestamp, so frames are
reproducible and distinguishable.

Used by tests, examples and the Studio demo to drive a synchronized robot + camera
session: attach it to a `SessionManager` (`attach_camera`) and every tick captures a
frame on the same clock as the robot telemetry, which the `SessionRecorder` streams
to the recording alongside `robot/telemetry` on the `camera/color` channel.

Swapping in a real `CameraAdapter` (file, OpenCV, vendor SDK) changes nothing else.
