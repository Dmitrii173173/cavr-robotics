#pragma once

// Neutral camera interface, mirroring ControllerAdapter: a concrete adapter (mock,
// file, OpenCV, vendor SDK, ...) produces time-stamped frames for a clock tick, so
// the runtime can capture a robot + camera stream synchronized on one clock. The
// rest of CAVR Studio stays camera-agnostic.

#include <cavr/adapter_sdk/camera_frame.hpp>
#include <cavr/core/time.hpp>

#include <optional>

namespace cavr::adapter_sdk {

class CameraAdapter {
 public:
  virtual ~CameraAdapter() = default;

  [[nodiscard]] virtual bool open() = 0;
  [[nodiscard]] virtual bool is_open() const = 0;
  virtual void close() = 0;

  // The frame available at this instant, or nullopt when no new frame is ready.
  [[nodiscard]] virtual std::optional<CameraFrame> poll(core::Timestamp now) = 0;
};

}  // namespace cavr::adapter_sdk
