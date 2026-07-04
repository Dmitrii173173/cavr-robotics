#pragma once

// Maps a stored robot's adapter key onto a concrete ControllerAdapter. This is the
// seam that makes the SDK universal from the registry's point of view: a robot row
// carries an adapter string ("mock", "generic_tcp", ...) and the UI turns it into a
// live controller here, instead of the choice being hard-wired to an env var.
//
// It lives at the app layer on purpose: the concrete adapters both depend on
// adapter_sdk, so the factory that constructs them must sit above both — never in
// adapter_sdk itself. cavr-studio already links every adapter it offers.

#include <cavr/adapter_sdk/controller_adapter.hpp>
#include <cavr/adapters/generic_tcp_robot/generic_tcp_controller.hpp>
#include <cavr/adapters/mock_robot/mock_controller.hpp>
#include <cavr/machine/machine_profile.hpp>

#include <memory>
#include <string_view>

namespace cavr::studio {

// Returns a controller for the given adapter key, or the mock for an unknown key
// (so a malformed registry row still yields a usable, safe default). The registry
// profile is injected into the mock so a stored robot (e.g. a PNR row on the mock
// backend) is simulated with its own kinematics and IO; a real TCP controller
// discovers its profile from the bridge, so the profile is not used there.
[[nodiscard]] inline std::unique_ptr<cavr::adapter_sdk::ControllerAdapter> make_adapter(
    std::string_view adapter, const cavr::machine::MachineProfile& profile) {
  if (adapter == "generic_tcp" || adapter == "tcp") {
    return std::make_unique<cavr::adapters::generic_tcp_robot::GenericTcpController>();
  }
  return std::make_unique<cavr::adapters::mock_robot::MockController>(profile);
}

}  // namespace cavr::studio
