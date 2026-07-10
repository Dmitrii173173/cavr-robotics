# File Camera Adapter

Replays a sequence of image files from disk as a `CameraAdapter`, for testing and
demos against a captured or hand-authored image sequence rather than
`MockCamera`'s synthetic pattern.

- **Formats (both dependency-free):**
  - binary [Netpbm](https://en.wikipedia.org/wiki/Netpbm) — `.pgm` (P5, 8-bit
    grayscale → `mono8`) and `.ppm` (P6, 8-bit RGB → `rgb8`). The pixel data is
    already raw bytes behind a tiny plain-text header.
    [`netpbm.hpp`](include/cavr/adapters/file_camera/netpbm.hpp) has a reader and
    writer.
  - `.png` via a **from-scratch PNG decoder**
    ([`png.hpp`](include/cavr/adapters/file_camera/png.hpp)) — a complete DEFLATE
    inflater (stored / fixed / dynamic Huffman, RFC 1951), PNG chunk parsing, and
    per-scanline unfiltering, with **no libpng/zlib/OpenCV**. Decodes 8-bit
    grayscale (→ `mono8`), truecolor (→ `rgb8`), indexed/palette (→ `rgb8`), and
    grayscale/truecolor + alpha (→ `rgba8`); 16-bit and interlaced PNGs are
    rejected rather than mis-decoded. A tiny stored-block `write_png` mirrors
    `write_pgm`/`write_ppm`.
- **`FileCameraAdapter`**
  ([`file_camera_adapter.hpp`](include/cavr/adapters/file_camera/file_camera_adapter.hpp)):
  construct from an explicit list of frame paths, or via
  `FileCameraAdapter::from_directory(dir, frame_id, fps)` to play back every
  `.pgm`/`.ppm`/`.png` file under `dir` in filename order (the decoder is chosen
  by extension). `poll(now)` paces frames against the timestamps it's given (not
  wall-clock time), matching `MockCamera`'s contract so it drops into
  `SessionManager::attach_camera` unchanged.

A live capture device (or a hardware-accelerated JPEG/H.264 path) is still out of
scope here; this adapter's job is dependency-free replay of a pre-captured or
synthetic sequence.
