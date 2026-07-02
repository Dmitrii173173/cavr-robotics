#pragma once

// Neutral controller interface. A concrete adapter (mock, generic TCP, vendor
// SDK, ...) implements this so the rest of CAVR Studio is controller-agnostic.
//
// Responsibilities:
//  * connect / disconnect to a physical or simulated controller;
//  * discover the machine profile (variables -> app structures);
//  * accept a planned MotionTask and start/pause/resume/stop it on the
//    controller (the controller, not the app, performs the motion);
//  * surface telemetry: poll() returns the latest RobotState for a clock tick.

#include <cavr/adapter_sdk/robot_state.hpp>
#include <cavr/core/time.hpp>
#include <cavr/machine/frames.hpp>
#include <cavr/machine/machine_profile.hpp>
#include <cavr/machine/motion.hpp>

#include <string>
#include <vector>

namespace cavr::adapter_sdk {

struct ConnectionInfo final {
  std::string endpoint;     // host:port, device path, or "mock"
  std::string transport;    // "tcp", "udp", "mock", ...
};

struct ConnectResult final {
  bool connected{false};
  std::vector<std::string> errors;
  [[nodiscard]] bool ok() const noexcept { return connected && errors.empty(); }
};

class ControllerAdapter {
 public:
  virtual ~ControllerAdapter() = default;

  [[nodiscard]] virtual ConnectResult connect(const ConnectionInfo& info) = 0;
  virtual void disconnect() = 0;
  [[nodiscard]] virtual bool is_connected() const = 0;

  // Read the machine capabilities from the controller into a profile.
  [[nodiscard]] virtual machine::MachineProfile discover_profile() const = 0;

  // Hand a planned task to the controller. Returns false if rejected.
  [[nodiscard]] virtual bool load_task(const machine::MotionTask& task) = 0;

  // Program control. These map to teach-pendant / controller commands.
  [[nodiscard]] virtual bool start() = 0;
  virtual void pause() = 0;
  virtual void resume() = 0;
  virtual void stop() = 0;

  // Immediate motion outside the loaded program: command a single move now — a
  // jog / teleoperation from the scene (the scene -> robot direction). Returns
  // false if the controller does not support live motion. Default: unsupported.
  [[nodiscard]] virtual bool move_to(const machine::MotionCommand& /*command*/) { return false; }

  // The controller's tool table (10 calibration slots) for reading — the current
  // selection and each slot's calibration. nullptr when unavailable. Mutations go
  // through select_tool/calibrate_tool/clear_tool so a remote controller stays in
  // sync (a bare pointer can't send the change over the wire).
  [[nodiscard]] virtual machine::ToolTable* tools() { return nullptr; }

  // Select the active tool (its offset defines the TCP). False if unsupported.
  [[nodiscard]] virtual bool select_tool(int /*slot*/) { return false; }
  // Calibrate a tool slot's TCP offset (pose in the flange frame).
  [[nodiscard]] virtual bool calibrate_tool(int /*slot*/, const core::Pose3D& /*tcp_offset*/) {
    return false;
  }
  // Clear a tool slot's calibration.
  [[nodiscard]] virtual bool clear_tool(int /*slot*/) { return false; }

  // Latest telemetry for the given wall-clock instant.
  [[nodiscard]] virtual RobotState poll(core::Timestamp now) = 0;
};

}  // namespace cavr::adapter_sdk
