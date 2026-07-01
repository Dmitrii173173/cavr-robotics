// GenericTcpController end-to-end against an in-process fake robot server that
// speaks the wire protocol over a real loopback TCP socket. Proves: the profile
// is pulled from the controller, a task is loaded and started, the pushed
// telemetry stream is drained by poll(), events are surfaced, and the whole
// thing is a drop-in ControllerAdapter for SessionManager (same as the mock).

#include <cavr/adapters/generic_tcp_robot/generic_tcp_controller.hpp>
#include <cavr/adapters/generic_tcp_robot/protocol.hpp>
#include <cavr/adapters/generic_tcp_robot/tcp_connection.hpp>
#include <cavr/adapters/mock_robot/mock_controller.hpp>
#include <cavr/machine/json.hpp>
#include <cavr/runtime/demo_plan.hpp>
#include <cavr/runtime/session_manager.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace {

int failures = 0;

void check(bool condition, std::string_view message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
    ++failures;
  }
}

namespace tcp = cavr::adapters::generic_tcp_robot;
namespace proto = cavr::adapters::generic_tcp_robot::protocol;
namespace json = cavr::json;
namespace machine = cavr::machine;
namespace sdk = cavr::adapter_sdk;
namespace runtime = cavr::runtime;

// A minimal robot controller server: accepts one connection, answers commands,
// and on "start" streams a short scripted trajectory ending in a Completed frame
// with a ProgramCompleted event. Regardless of the task it receives, so a test
// gets deterministic telemetry.
class FakeRobotServer {
 public:
  [[nodiscard]] std::string start(std::uint16_t frames) {
    frames_ = frames;
    if (std::string error = listener_.listen(0); !error.empty()) return error;
    port_ = listener_.port();
    thread_ = std::thread([this] { serve(); });
    return {};
  }

  [[nodiscard]] std::uint16_t port() const noexcept { return port_; }

  void join() {
    if (thread_.joinable()) thread_.join();
  }

  [[nodiscard]] bool saw_task() const noexcept { return saw_task_.load(); }

 private:
  static void ack(tcp::TcpConnection& conn, const std::string& cmd) {
    json::Value r;
    r.set("type", "ack");
    r.set("cmd", cmd);
    r.set("ok", true);
    (void)conn.send_line(r.dump(0));
  }

  void stream_frames(tcp::TcpConnection& conn) {
    for (std::uint16_t i = 0; i < frames_; ++i) {
      sdk::RobotState s;
      s.timestamp = cavr::core::Timestamp::from_nanoseconds(static_cast<std::int64_t>(i) * 20'000'000);
      s.joint_positions = {0.01 * i, 0.02 * i, -0.01 * i, 0.0, 0.03 * i, 0.0};
      s.servo_state = machine::ServoState::On;
      s.current_step = i;
      s.current_step_label = "step " + std::to_string(i);
      s.speed_fraction = 1.0;
      const bool last = (i + 1 == frames_);
      s.program_state = last ? machine::ProgramState::Completed : machine::ProgramState::Running;
      if (last) {
        s.events.push_back({s.timestamp, machine::EventKind::ProgramCompleted, machine::Severity::Info,
                            "Program completed"});
      }
      json::Value msg = proto::state_to_json(s);
      msg.set("type", "state");
      (void)conn.send_line(msg.dump(0));
    }
  }

  void serve() {
    tcp::TcpConnection conn = listener_.accept(3000);
    if (!conn.is_open()) return;

    bool running = true;
    while (running) {
      std::string line;
      bool timed_out = false;
      if (!conn.read_line(3000, line, timed_out)) break;
      std::string error;
      auto value = json::parse(line, error);
      if (!value) continue;
      const std::string cmd = value->at("cmd").as_string();
      if (cmd == "discover_profile") {
        json::Value r;
        r.set("type", "profile");
        r.set("profile", machine::to_json(cavr::adapters::mock_robot::make_gp25_profile()));
        (void)conn.send_line(r.dump(0));
      } else if (cmd == "load_task") {
        saw_task_ = value->at("task").is_array() && !value->at("task").as_array().empty();
        ack(conn, "load_task");
      } else if (cmd == "start") {
        ack(conn, "start");
        stream_frames(conn);
      } else if (cmd == "stop") {
        ack(conn, "stop");
        running = false;
      } else {
        ack(conn, cmd);
      }
    }
  }

  tcp::TcpListener listener_;
  std::thread thread_;
  std::uint16_t port_{0};
  std::uint16_t frames_{5};
  std::atomic<bool> saw_task_{false};
};

std::string endpoint(std::uint16_t port) { return "127.0.0.1:" + std::to_string(port); }

// Drive the controller directly through the adapter interface.
void test_direct_protocol() {
  FakeRobotServer server;
  check(server.start(/*frames=*/5).empty(), "fake server starts listening");

  tcp::GenericTcpController controller;
  const auto result = controller.connect({endpoint(server.port()), "tcp"});
  check(result.ok(), "controller connects over TCP");
  check(controller.is_connected(), "controller reports connected");

  const machine::MachineProfile profile = controller.discover_profile();
  check(profile.robot_model == "Yaskawa Motoman GP25", "profile discovered from controller");
  check(profile.axes.size() == 6, "discovered profile has six axes");

  check(controller.load_task(runtime::make_demo_plan().to_motion_task()), "task loads");
  check(controller.start(), "program starts");

  // Poll until the controller reports Completed (telemetry is server-pushed).
  sdk::RobotState latest;
  bool completed = false;
  bool saw_completed_event = false;
  std::int64_t now_ns = 0;
  for (int i = 0; i < 500 && !completed; ++i) {
    latest = controller.poll(cavr::core::Timestamp::from_nanoseconds(now_ns));
    now_ns += 20'000'000;
    for (const auto& e : latest.events) {
      if (e.kind == machine::EventKind::ProgramCompleted) saw_completed_event = true;
    }
    if (latest.program_state == machine::ProgramState::Completed) completed = true;
    else std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  check(completed, "controller reaches Completed from the pushed stream");
  check(saw_completed_event, "ProgramCompleted event surfaced through poll()");
  check(latest.joint_positions.size() == 6, "telemetry carries six joint values");
  check(server.saw_task(), "server received a non-empty task");

  controller.stop();
  server.join();
}

// Prove the adapter is a drop-in for SessionManager, exactly like MockController.
void test_session_manager_integration() {
  FakeRobotServer server;
  check(server.start(/*frames=*/8).empty(), "fake server (session) starts");

  tcp::GenericTcpController controller;
  runtime::SessionManager manager;

  const auto connect = manager.connect(controller, {endpoint(server.port()), "tcp"});
  check(connect.ok(), "session manager connects the TCP controller");
  (void)manager.discover_profile();
  manager.set_plan(runtime::make_demo_plan());
  const auto report = manager.validate();
  check(report.ok(), "demo plan validates against the discovered profile");
  check(manager.execute("tcp_session"), "session executes");

  std::int64_t now_ns = 1'000'000'000;
  for (int i = 0; i < 500 && manager.phase() == runtime::SessionPhase::Executing; ++i) {
    manager.tick(cavr::core::Timestamp::from_nanoseconds(now_ns));
    now_ns += 20'000'000;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  check(manager.phase() == runtime::SessionPhase::Completed, "session reaches Completed");
  check(!manager.log().frames.empty(), "session recorded telemetry frames");

  manager.stop();  // triggers the server's stop handler
  server.join();
}

// A bad endpoint fails cleanly rather than hanging or crashing.
void test_connect_failure() {
  tcp::GenericTcpController controller;
  const auto result = controller.connect({"127.0.0.1:1", "tcp"});  // nothing listening on port 1
  check(!result.ok(), "connecting to a dead endpoint fails");
  check(!controller.is_connected(), "failed connect leaves controller disconnected");

  const auto bad = controller.connect({"not-an-endpoint", "tcp"});
  check(!bad.ok(), "malformed endpoint is rejected");
}

}  // namespace

int main() {
  test_direct_protocol();
  test_session_manager_integration();
  test_connect_failure();

  if (failures != 0) {
    std::cerr << failures << " generic tcp robot test(s) failed\n";
    return 1;
  }
  std::cout << "generic tcp robot tests passed\n";
  return 0;
}
