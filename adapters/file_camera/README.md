# File Camera Adapter

Replays a sequence of image files from disk as a `CameraAdapter`, for testing and
demos against a captured or hand-authored image sequence rather than
`MockCamera`'s synthetic pattern.

- **Format:** binary [Netpbm](https://en.wikipedia.org/wiki/Netpbm) — `.pgm`
  (P5, 8-bit grayscale → `mono8`) and `.ppm` (P6, 8-bit RGB → `rgb8`). Chosen
  because it needs no image-decoding dependency: the pixel data is already raw
  bytes in the layout `CameraFrame` wants, behind a tiny plain-text header.
  [`netpbm.hpp`](include/cavr/adapters/file_camera/netpbm.hpp) has a
  dependency-free reader and writer.
- **`FileCameraAdapter`**
  ([`file_camera_adapter.hpp`](include/cavr/adapters/file_camera/file_camera_adapter.hpp)):
  construct from an explicit list of frame paths, or via
  `FileCameraAdapter::from_directory(dir, frame_id, fps)` to play back every
  `.pgm`/`.ppm` file under `dir` in filename order. `poll(now)` paces frames
  against the timestamps it's given (not wall-clock time), matching
  `MockCamera`'s contract so it drops into `SessionManager::attach_camera`
  unchanged.

A real image-decoding adapter (PNG/JPEG via an external library, or a live
capture device) is out of scope here; this adapter's job is dependency-free
replay of a pre-captured or synthetic sequence.
