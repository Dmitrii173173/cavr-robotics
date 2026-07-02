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
namespace mock = cavr::adapters::mock_robot;
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
      } else if (cmd == "get_tools") {
        machine::ToolTable tools;  // default table; enough to satisfy discover
        json::Value r = proto::tools_to_json(tools);
        r.set("type", "tools");
        (void)conn.send_line(r.dump(0));
      } else if (cmd == "load_task") {
        saw_task_ = value->at("task").is_array() && !value->at("task").as_array().empty();
        ack(conn, "load_task");
      } else if (cmd == "start") {
        ack(conn, "start");
        stream_frames(conn);
      } else if (cmd == "move_to") {
        // Apply the jog: acknowledge, then stream one state frame carrying the
        // commanded joints, so the client sees the robot reach the target.
        const machine::MotionCommand jog = proto::command_from_json(value->at("command"));
        ack(conn, "move_to");
        cavr::adapter_sdk::RobotState s;
        s.timestamp = cavr::core::Timestamp::from_nanoseconds(0);
        if (jog.target.joints) s.joint_positions = *jog.target.joints;
        s.program_state = machine::ProgramState::Running;
        s.servo_state = machine::ServoState::On;
        s.current_step_label = jog.label;
        json::Value msg = proto::state_to_json(s);
        msg.set("type", "state");
        (void)conn.send_line(msg.dump(0));
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

// Scene -> robot: an immediate jog command travels to the controller and the
// telemetry reflects the commanded joint target.
void test_move_to_jog() {
  FakeRobotServer server;
  check(server.start(/*frames=*/5).empty(), "fake server (jog) starts");

  tcp::GenericTcpController controller;
  check(controller.connect({endpoint(server.port()), "tcp"}).ok(), "controller connects for jog");

  machine::MotionCommand jog;
  jog.kind = machine::MotionKind::MoveJ;
  const std::vector<double> target = {0.11, 0.22, -0.33, 0.44, 0.55, -0.66};
  jog.target.joints = target;
  jog.label = "jog target";
  check(controller.move_to(jog), "move_to is acknowledged");

  // Poll until the telemetry carries the commanded joints (the server echoes them).
  bool reached = false;
  std::int64_t now_ns = 0;
  for (int i = 0; i < 200 && !reached; ++i) {
    const sdk::RobotState s = controller.poll(cavr::core::Timestamp::from_nanoseconds(now_ns));
    now_ns += 20'000'000;
    if (s.joint_positions.size() == target.size()) {
      bool match = true;
      for (std::size_t j = 0; j < target.size(); ++j) {
        if (std::abs(s.joint_positions[j] - target[j]) > 1e-6) match = false;
      }
      reached = match;
    }
    if (!reached) std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  check(reached, "robot telemetry reaches the jogged joint target");

  controller.stop();
  server.join();
}

// The mock controller's own live jog: from home, move_to interpolates to the
// commanded joints and poll() eventually reports them.
void test_mock_move_to() {
  mock::MockController controller;
  (void)controller.connect({"mock", "mock"});

  machine::MotionCommand jog;
  jog.kind = machine::MotionKind::MoveJ;
  const std::vector<double> target = {0.2, -0.1, 0.15, 0.0, 0.3, 0.0};
  jog.target.joints = target;
  jog.speed = 3.0;  // rad/s
  check(controller.move_to(jog), "mock accepts the jog");

  sdk::RobotState s;
  std::int64_t now_ns = 0;
  for (int i = 0; i < 500; ++i) {
    s = controller.poll(cavr::core::Timestamp::from_nanoseconds(now_ns));
    now_ns += 20'000'000;
  }
  bool reached = s.joint_positions.size() == target.size();
  for (std::size_t j = 0; reached && j < target.size(); ++j) {
    if (std::abs(s.joint_positions[j] - target[j]) > 1e-3) reached = false;
  }
  check(reached, "mock jog reaches the commanded joint target");
}

// Cartesian jog on the mock: move_to a TCP pose (not joints) is solved through
// inverse kinematics, and the reached pose matches the commanded target.
void test_mock_cartesian_move_to() {
  mock::MockController controller;
  (void)controller.connect({"mock", "mock"});

  // A reachable pose: take FK of a known joint config as the Cartesian target.
  const auto profile = mock::make_gp25_profile();
  const std::vector<double> ref = {0.3, 0.2, -0.2, 0.0, 0.4, 0.0};
  const cavr::core::Pose3D target =
      cavr::machine::forward_kinematics(profile.axes, ref, cavr::core::Vec3{0, 0, 0.101}).tcp;

  machine::MotionCommand jog;
  jog.kind = machine::MotionKind::MoveL;
  jog.target.pose = target;
  jog.speed = 500.0;  // mm/s (Cartesian speed)
  check(controller.move_to(jog), "mock accepts a Cartesian jog (IK converges)");

  sdk::RobotState s;
  std::int64_t now_ns = 0;
  for (int i = 0; i < 800; ++i) {
    s = controller.poll(cavr::core::Timestamp::from_nanoseconds(now_ns));
    now_ns += 20'000'000;
  }
  const double dx = s.tcp_pose.position_m.x_m - target.position_m.x_m;
  const double dy = s.tcp_pose.position_m.y_m - target.position_m.y_m;
  const double dz = s.tcp_pose.position_m.z_m - target.position_m.z_m;
  check(std::sqrt(dx * dx + dy * dy + dz * dz) < 2.0e-3,
        "mock Cartesian jog reaches the commanded TCP position");
}

// A calibrated tool changes the TCP: extending the tool along Z pushes the
// reported TCP further from the flange by the tool length.
void test_tool_offset_moves_tcp() {
  mock::MockController controller;
  (void)controller.connect({"mock", "mock"});
  machine::ToolTable* tools = controller.tools();
  check(tools != nullptr, "mock exposes a tool table");
  if (!tools) return;
  check(tools->size() == 10, "tool table has 10 slots");

  const sdk::RobotState with_flange = controller.poll(cavr::core::Timestamp::from_nanoseconds(0));

  // Calibrate slot 1 as a 0.3 m tool along Z and select it.
  tools->set_tool(1, cavr::core::Pose3D{cavr::core::Vec3{0, 0, 0.3}, cavr::core::Quaternion::identity()},
                  "long torch");
  check(tools->select(1), "select the calibrated tool");
  const sdk::RobotState with_tool = controller.poll(cavr::core::Timestamp::from_nanoseconds(20'000'000));

  // At home the arm points up, so a longer tool raises the TCP height. Either way
  // the TCP must move by ~0.2 m (0.3 − the 0.101 flange tool) relative to before.
  const double dz = with_tool.tcp_pose.position_m.z_m - with_flange.tcp_pose.position_m.z_m;
  check(std::abs(dz - 0.199) < 1e-3, "selecting a longer tool moves the TCP by the tool-length delta");
}

// Tool table over TCP: calibrating and selecting a tool on the remote controller
// (through the protocol) changes the remote robot's reported TCP, and the client
// mirror reflects the calibration.
void test_tools_over_tcp() {
  tcp::TcpListener listener;
  check(listener.listen(0).empty(), "tool server listens");
  const std::uint16_t port = listener.port();

  std::thread server([&listener] {
    tcp::TcpConnection conn = listener.accept(3000);
    if (!conn.is_open()) return;
    mock::MockController mc;
    (void)mc.connect({"mock", "mock"});

    auto ack = [&conn](const std::string& c) {
      json::Value r;
      r.set("type", "ack");
      r.set("cmd", c);
      r.set("ok", true);
      (void)conn.send_line(r.dump(0));
    };

    // Phase 1: answer setup commands until the client selects a tool.
    bool ready_to_stream = false;
    while (!ready_to_stream) {
      std::string line;
      bool timed_out = false;
      if (!conn.read_line(3000, line, timed_out)) return;
      std::string error;
      auto v = json::parse(line, error);
      if (!v) continue;
      const std::string cmd = v->at("cmd").as_string();
      if (cmd == "discover_profile") {
        json::Value r;
        r.set("type", "profile");
        r.set("profile", machine::to_json(mock::make_gp25_profile()));
        (void)conn.send_line(r.dump(0));
      } else if (cmd == "get_tools") {
        json::Value r = proto::tools_to_json(*mc.tools());
        r.set("type", "tools");
        (void)conn.send_line(r.dump(0));
      } else if (cmd == "calibrate_tool") {
        (void)mc.calibrate_tool(static_cast<int>(v->at("slot").as_int()),
                                cavr::machine::detail::pose_from_json(v->at("tcp")));
        ack("calibrate_tool");
      } else if (cmd == "select_tool") {
        (void)mc.select_tool(static_cast<int>(v->at("slot").as_int()));
        ack("select_tool");
        ready_to_stream = true;
      }
    }
    // Phase 2: stream the (stationary) home pose so the client reads the TCP that
    // the freshly selected tool produces.
    std::int64_t t = 0;
    for (int i = 0; i < 60 && conn.is_open(); ++i) {
      const sdk::RobotState s = mc.poll(cavr::core::Timestamp::from_nanoseconds(t));
      t += 20'000'000;
      json::Value msg = proto::state_to_json(s);
      msg.set("type", "state");
      if (!conn.send_line(msg.dump(0)).empty()) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  });

  tcp::GenericTcpController controller;
  check(controller.connect({endpoint(port), "tcp"}).ok(), "controller connects for tools");
  (void)controller.discover_profile();  // also pulls the tool mirror (tool 0 calibrated)

  machine::ToolTable* mirror = controller.tools();
  check(mirror != nullptr, "TCP controller exposes a tool mirror");
  check(mirror && mirror->slot(0).calibrated, "mirror shows the pre-calibrated flange tool");

  // Calibrate a 0.3 m tool remotely and select it.
  check(controller.calibrate_tool(1, cavr::core::Pose3D{cavr::core::Vec3{0, 0, 0.3},
                                                        cavr::core::Quaternion::identity()}),
        "remote calibrate_tool acked");
  check(controller.select_tool(1), "remote select_tool acked");
  check(mirror && mirror->slot(1).calibrated && mirror->current() == 1,
        "client mirror reflects the remote calibration + selection");

  // Home TCP height with the default 0.101 m tool vs the 0.3 m tool: the streamed
  // TCP must sit ~0.199 m higher, proving the remote tool change took effect.
  const auto axes = mock::make_gp25_profile().axes;
  const double base_z = machine::forward_kinematics(axes, {0, 0, 0, 0, 0, 0},
                                                     cavr::core::Vec3{0, 0, 0.101}).tcp.position_m.z_m;
  double best_z = 0.0;
  std::int64_t now_ns = 0;
  for (int i = 0; i < 200; ++i) {
    const sdk::RobotState s = controller.poll(cavr::core::Timestamp::from_nanoseconds(now_ns));
    now_ns += 20'000'000;
    best_z = std::max(best_z, s.tcp_pose.position_m.z_m);
    if (best_z > base_z + 0.15) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  check(best_z > base_z + 0.15, "remote TCP reflects the calibrated long tool");

  controller.stop();
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
  test_move_to_jog();
  test_mock_move_to();
  test_mock_cartesian_move_to();
  test_tool_offset_moves_tcp();
  test_tools_over_tcp();
  test_connect_failure();

  if (failures != 0) {
    std::cerr << failures << " generic tcp robot test(s) failed\n";
    return 1;
  }
  std::cout << "generic tcp robot tests passed\n";
  return 0;
}
