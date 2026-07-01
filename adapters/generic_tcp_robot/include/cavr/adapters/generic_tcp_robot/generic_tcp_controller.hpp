#pragma once

// A real ControllerAdapter that speaks the CAVR generic TCP robot protocol
// (protocol.hpp) to a controller bridge/PLC over a socket. It is a drop-in
// replacement for MockController: connect to a host:port instead of "mock", and
// nothing else in the runtime or Studio changes.
//
// Telemetry is a server-pushed stream. poll() is non-blocking — it drains
// whatever telemetry has arrived since the last tick and returns the latest
// frame, stamped with the caller's clock so it stays aligned with the session
// clock and any synchronized camera. Program-control commands (load/start/pause/
// resume/stop) are request/ack round trips; telemetry that arrives while awaiting
// an ack is buffered, never dropped.

#include <cavr/adapter_sdk/controller_adapter.hpp>
#include <cavr/adapters/generic_tcp_robot/protocol.hpp>
#include <cavr/adapters/generic_tcp_robot/tcp_connection.hpp>
#include <cavr/machine/json.hpp>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace cavr::adapters::generic_tcp_robot {

namespace machine = cavr::machine;
namespace sdk = cavr::adapter_sdk;

class GenericTcpController final : public sdk::ControllerAdapter {
 public:
  // Milliseconds to wait for a command acknowledgement / the profile reply.
  explicit GenericTcpController(int reply_timeout_ms = 3000) : reply_timeout_ms_(reply_timeout_ms) {}

  // Connects to info.endpoint ("host:port"). transport, if set, must be "tcp".
  [[nodiscard]] sdk::ConnectResult connect(const sdk::ConnectionInfo& info) override {
    sdk::ConnectResult result;
    if (!info.transport.empty() && info.transport != "tcp") {
      result.errors.push_back("GenericTcpController requires tcp transport, got: " + info.transport);
      return result;
    }
    std::string host;
    std::uint16_t port = 0;
    if (!parse_endpoint(info.endpoint, host, port)) {
      result.errors.push_back("Invalid endpoint (expected host:port): " + info.endpoint);
      return result;
    }
    if (std::string error = conn_.connect(host, port); !error.empty()) {
      result.errors.push_back(error);
      return result;
    }
    connected_ = true;
    result.connected = true;
    return result;
  }

  void disconnect() override {
    conn_.close();
    connected_ = false;
    started_ = false;
    inbox_.clear();
  }

  [[nodiscard]] bool is_connected() const override { return connected_ && conn_.is_open(); }

  [[nodiscard]] machine::MachineProfile discover_profile() const override {
    if (!conn_.is_open()) return profile_;
    if (!conn_.send_line(protocol::command_line("discover_profile")).empty()) return profile_;
    if (auto reply = read_reply("profile"); reply) {
      profile_ = machine::profile_from_json(reply->at("profile"));
    }
    return profile_;
  }

  [[nodiscard]] bool load_task(const machine::MotionTask& task) override {
    if (!is_connected()) return false;
    if (!conn_.send_line(protocol::load_task_line(task)).empty()) return false;
    return ack_ok("load_task");
  }

  [[nodiscard]] bool start() override {
    if (!is_connected()) return false;
    if (!conn_.send_line(protocol::command_line("start")).empty()) return false;
    started_ = ack_ok("start");
    return started_;
  }

  void pause() override { send_best_effort("pause"); }
  void resume() override { send_best_effort("resume"); }
  void stop() override {
    send_best_effort("stop");
    started_ = false;
  }

  // Immediate move: send a single motion command for the controller to run now
  // (jog / teleoperation from the scene). Returns whether it was acknowledged.
  [[nodiscard]] bool move_to(const machine::MotionCommand& command) override {
    if (!is_connected()) return false;
    if (!conn_.send_line(protocol::move_to_line(command)).empty()) return false;
    return ack_ok("move_to");
  }

  [[nodiscard]] sdk::RobotState poll(core::Timestamp now) override {
    std::vector<sdk::ControllerEvent> pending;

    // Telemetry buffered while awaiting an ack is processed first, in order.
    for (const auto& line : inbox_) apply_state_line(line, pending);
    inbox_.clear();

    std::vector<std::string> lines;
    const bool open = conn_.drain_lines(lines);
    for (const auto& line : lines) apply_state_line(line, pending);

    if (!open && connected_) {
      connected_ = false;  // peer closed the stream
      if (last_.error_severity != machine::Severity::Error) {
        last_.error_severity = machine::Severity::Error;
        last_.error_message = "Controller connection closed";
      }
    }

    sdk::RobotState out = last_;
    out.timestamp = now;
    out.events = std::move(pending);  // only events new this tick, like the mock
    return out;
  }

 private:
  // Parses a "host:port" endpoint. Rejects empty host, missing/invalid port.
  [[nodiscard]] static bool parse_endpoint(const std::string& endpoint, std::string& host,
                                           std::uint16_t& port) {
    const auto colon = endpoint.rfind(':');
    if (colon == std::string::npos || colon == 0 || colon + 1 >= endpoint.size()) return false;
    host = endpoint.substr(0, colon);
    const std::string port_str = endpoint.substr(colon + 1);
    for (char c : port_str) {
      if (c < '0' || c > '9') return false;
    }
    const long value = std::stol(port_str);
    if (value <= 0 || value > 65535) return false;
    port = static_cast<std::uint16_t>(value);
    return true;
  }

  // Reads lines until one of the given `type` arrives (returning it) or the read
  // times out / the peer closes (returning nullopt). Any "state" telemetry seen
  // while waiting is stashed in inbox_ for the next poll(), never dropped.
  [[nodiscard]] std::optional<json::Value> read_reply(std::string_view type) const {
    for (;;) {
      std::string line;
      bool timed_out = false;
      if (!conn_.read_line(reply_timeout_ms_, line, timed_out)) return std::nullopt;
      std::string error;
      auto value = json::parse(line, error);
      if (!value) continue;
      if (protocol::message_type(*value) == type) return value;
      inbox_.push_back(line);  // telemetry interleaved with the reply
    }
  }

  // Sends a command and returns whether the ack for it is ok.
  [[nodiscard]] bool ack_ok(std::string_view cmd) {
    auto reply = read_reply("ack");
    return reply && reply->at("ok").as_bool() && reply->at("cmd").as_string() == cmd;
  }

  // Sends a command and burns its ack without requiring success (pause/resume/stop).
  void send_best_effort(std::string_view cmd) {
    if (!is_connected()) return;
    if (!conn_.send_line(protocol::command_line(cmd)).empty()) return;
    (void)read_reply("ack");
  }

  // Parses one telemetry line; a "state" updates last_ and appends its events.
  void apply_state_line(const std::string& line, std::vector<sdk::ControllerEvent>& pending) {
    std::string error;
    auto value = json::parse(line, error);
    if (!value) return;
    if (protocol::message_type(*value) != "state") return;
    last_ = protocol::state_from_json(*value);
    for (const auto& e : last_.events) pending.push_back(e);
  }

  int reply_timeout_ms_;
  mutable TcpConnection conn_;
  mutable machine::MachineProfile profile_;
  mutable std::vector<std::string> inbox_;  // telemetry lines awaiting the next poll
  sdk::RobotState last_;
  bool connected_{false};
  bool started_{false};
};

}  // namespace cavr::adapters::generic_tcp_robot
