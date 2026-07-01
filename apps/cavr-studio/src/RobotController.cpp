#include "RobotController.hpp"

#include <cavr/runtime/demo_plan.hpp>
#include <cavr/runtime/session_io.hpp>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <vector>

namespace {
constexpr std::int64_t kTickNs = 20'000'000;  // 50 Hz simulated clock
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
}  // namespace

RobotController::RobotController(QObject* parent) : QObject(parent) {
  // Pick the robot source. CAVR_ROBOT_ENDPOINT=host:port drives the scene from a
  // remote robot over TCP (a cavr-robotd or a vendor bridge) — the robot -> scene
  // digital-twin path; otherwise the in-process mock keeps the standalone demo.
  cavr::adapter_sdk::ConnectionInfo info{"mock", "mock"};
  if (const char* endpoint = std::getenv("CAVR_ROBOT_ENDPOINT"); endpoint && *endpoint) {
    controller_ = std::make_unique<cavr::adapters::generic_tcp_robot::GenericTcpController>();
    remote_ = true;
    info = {endpoint, "tcp"};
  } else {
    controller_ = std::make_unique<cavr::adapters::mock_robot::MockController>();
  }

  // connect -> discover profile -> plan -> validate -> execute
  static_cast<void>(manager_.connect(*controller_, info));
  static_cast<void>(manager_.discover_profile());
  manager_.set_plan(cavr::runtime::make_demo_plan());
  static_cast<void>(manager_.validate());
  static_cast<void>(manager_.execute("studio_session_0"));

  connect(&timer_, &QTimer::timeout, this, &RobotController::tick);
  timer_.start(20);
  publish();
}

void RobotController::tick() {
  now_ns_ += kTickNs;
  manager_.tick(cavr::core::Timestamp::from_nanoseconds(now_ns_));

  for (const auto& e : manager_.latest().events) {
    emit eventLogged(QString::fromStdString(cavr::machine::to_string(e.kind) + " | " + e.message));
  }

  // For the in-process mock, loop the demo so the cell keeps running like a
  // repeating production cycle. A remote robot drives its own motion (cavr-robotd
  // loops continuously), so we just keep mirroring its telemetry.
  if (!remote_ && manager_.phase() == cavr::runtime::SessionPhase::Completed) {
    ++run_index_;
    manager_.set_plan(cavr::runtime::make_demo_plan());
    static_cast<void>(manager_.validate());
    static_cast<void>(manager_.execute("studio_session_" + std::to_string(run_index_)));
  }

  publish();
}

void RobotController::publish() {
  const auto& s = manager_.latest();

  QVariantList joints;
  for (double q : s.joint_positions) joints.push_back(q * kRadToDeg);
  joint_degrees_ = joints;

  tcp_position_ = QVariantList{s.tcp_pose.position_m.x_m, s.tcp_pose.position_m.y_m,
                               s.tcp_pose.position_m.z_m};

  phase_ = QString::fromStdString(cavr::runtime::to_string(manager_.phase()));
  program_state_ = QString::fromStdString(cavr::machine::to_string(s.program_state));
  step_label_ = QString::fromStdString(s.current_step_label);
  speed_fraction_ = s.speed_fraction;

  weld_active_ = false;
  for (const auto& io : s.io) {
    if (io.name == "weld_on" && io.value > 0.5) weld_active_ = true;
  }

  emit telemetryChanged();
  emit phaseChanged(phase_);
}

void RobotController::start() {
  static_cast<void>(manager_.execute("studio_session_" + std::to_string(++run_index_)));
  publish();
}
void RobotController::pause() { controller_->pause(); }
void RobotController::resume() { controller_->resume(); }
void RobotController::stop() {
  manager_.stop();
  publish();
}

bool RobotController::saveSession(const QString& path) {
  std::vector<std::string> errors;
  return cavr::runtime::save_session_log(manager_.log(), path.toStdString(), errors);
}
