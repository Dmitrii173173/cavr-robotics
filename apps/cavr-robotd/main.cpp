// cavr-robotd: a reference robot server that speaks the generic_tcp_robot wire
// protocol over TCP, so a CAVR client (Studio, cavr-record) can drive and monitor
// a robot across the network without a vendor SDK on the client side. It stands in
// for the per-vendor "bridge" in the universal-adapter architecture: a real bridge
// would translate the same protocol to a vendor controller (e.g. a PNR/Crp SDK)
// running next to the robot; this one is backed by the deterministic MockController
// so the whole channel — profile discovery, task load, live telemetry — can be
// exercised end to end on any platform.
//
// It continuously loops the demo welding trajectory while a client is executing,
// so the robot keeps moving: connect CAVR Studio to it and the virtual GP25
// mirrors this "remote robot" live (the robot -> scene digital-twin direction).

#include <cavr/adapters/generic_tcp_robot/protocol.hpp>
#include <cavr/adapters/generic_tcp_robot/tcp_connection.hpp>
#include <cavr/adapters/mock_robot/mock_controller.hpp>
#include <cavr/machine/json.hpp>
#include <cavr/runtime/demo_plan.hpp>

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace {

namespace tcp = cavr::adapters::generic_tcp_robot;
namespace proto = cavr::adapters::generic_tcp_robot::protocol;
namespace mock = cavr::adapters::mock_robot;
namespace machine = cavr::machine;
namespace runtime = cavr::runtime;
namespace json = cavr::json;

constexpr std::int64_t kTickNs = 20'000'000;  // 50 Hz telemetry

void print_usage() {
  std::cout <<
      "Usage: cavr-robotd [--port <n>] [--rate-hz <n>]\n\n"
      "A reference robot server speaking the CAVR generic TCP robot protocol,\n"
      "backed by the deterministic mock GP25. A CAVR client (Studio, cavr-record)\n"
      "connects and receives a live, looping welding trajectory as telemetry.\n\n"
      "  --port <n>      TCP port to listen on. Default: 9010.\n"
      "  --rate-hz <n>   Telemetry frames per second while running. Default: 50.\n";
}

void send_ack(tcp::TcpConnection& conn, const std::string& cmd) {
  json::Value r;
  r.set("type", "ack");
  r.set("cmd", cmd);
  r.set("ok", true);
  (void)conn.send_line(r.dump(0));
}

void send_state(tcp::TcpConnection& conn, const cavr::adapter_sdk::RobotState& s) {
  json::Value msg = proto::state_to_json(s);
  msg.set("type", "state");
  (void)conn.send_line(msg.dump(0));
}

// Streams the mock's trajectory to the client until it disconnects or sends stop,
// looping the demo so the robot keeps moving. Completed frames are folded back
// into a restart, so the client sees an uninterrupted Running stream.
void stream_until_stopped(tcp::TcpConnection& conn, mock::MockController& controller, int rate_hz) {
  const auto frame_period = std::chrono::milliseconds(1000 / (rate_hz > 0 ? rate_hz : 50));
  std::int64_t now_ns = 0;
  bool running = true;
  while (running && conn.is_open()) {
    cavr::adapter_sdk::RobotState s = controller.poll(cavr::core::Timestamp::from_nanoseconds(now_ns));
    if (s.program_state == machine::ProgramState::Completed) {
      (void)controller.start();  // loop the trajectory; client keeps seeing Running
      now_ns += kTickNs;
      continue;
    }
    send_state(conn, s);

    // Drain any client commands that arrived (stop/pause) without blocking.
    std::vector<std::string> lines;
    if (!conn.drain_lines(lines)) break;  // client disconnected
    for (const auto& line : lines) {
      std::string error;
      auto value = json::parse(line, error);
      if (!value) continue;
      const std::string cmd = value->at("cmd").as_string();
      if (cmd == "stop") {
        send_ack(conn, "stop");
        running = false;
      } else if (cmd == "pause" || cmd == "resume") {
        send_ack(conn, cmd);  // acknowledged; the demo stream keeps flowing
      }
    }

    now_ns += kTickNs;
    std::this_thread::sleep_for(frame_period);
  }
}

// Serves one client connection through the command/telemetry protocol.
void serve_client(tcp::TcpConnection& conn, int rate_hz) {
  mock::MockController controller;
  (void)controller.connect({"mock", "mock"});  // establishes the GP25 profile
  bool task_loaded = false;

  while (conn.is_open()) {
    std::string line;
    bool timed_out = false;
    if (!conn.read_line(60'000, line, timed_out)) {
      if (timed_out) continue;
      break;  // client disconnected
    }
    std::string error;
    auto value = json::parse(line, error);
    if (!value) continue;
    const std::string cmd = value->at("cmd").as_string();

    if (cmd == "discover_profile") {
      json::Value r;
      r.set("type", "profile");
      r.set("profile", machine::to_json(controller.discover_profile()));
      (void)conn.send_line(r.dump(0));
    } else if (cmd == "load_task") {
      const machine::MotionTask task = proto::task_from_json(value->at("task"));
      task_loaded = controller.load_task(task);
      send_ack(conn, "load_task");
    } else if (cmd == "start") {
      if (!task_loaded) {
        (void)controller.load_task(runtime::make_demo_plan().to_motion_task());
        task_loaded = true;
      }
      (void)controller.start();
      send_ack(conn, "start");
      stream_until_stopped(conn, controller, rate_hz);
    } else {
      send_ack(conn, cmd);
    }
  }
}

}  // namespace

int main(int argc, char** argv) {
  std::uint16_t port = 9010;
  int rate_hz = 50;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      print_usage();
      return 0;
    } else if (arg == "--port" && i + 1 < argc) {
      port = static_cast<std::uint16_t>(std::atoi(argv[++i]));
    } else if (arg == "--rate-hz" && i + 1 < argc) {
      rate_hz = std::atoi(argv[++i]);
    } else {
      std::cerr << "error: unrecognized argument '" << arg << "'\n";
      print_usage();
      return 2;
    }
  }

  tcp::TcpListener listener;
  if (std::string error = listener.listen(port); !error.empty()) {
    std::cerr << "error: " << error << '\n';
    return 1;
  }
  std::cout << "cavr-robotd listening on 127.0.0.1:" << listener.port()
            << " (mock GP25, " << rate_hz << " Hz). Ctrl-C to stop.\n";

  // Serve clients sequentially; one robot, one controlling client at a time.
  for (;;) {
    tcp::TcpConnection conn = listener.accept(/*timeout_ms=*/-1);
    if (!conn.is_open()) continue;
    std::cout << "client connected\n";
    serve_client(conn, rate_hz);
    std::cout << "client disconnected\n";
  }
}
