#include "RobotController.hpp"

#include "AdapterFactory.hpp"

#include <cavr/machine/enums.hpp>
#include <cavr/runtime/demo_plan.hpp>
#include <cavr/runtime/session_io.hpp>

#include <QDir>
#include <QStandardPaths>
#include <QVariantMap>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

namespace {
constexpr std::int64_t kTickNs = 20'000'000;  // 50 Hz simulated clock
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

// Fixed-axis (Rx, Ry, Rz) Euler angles in degrees from a quaternion — the pendant
// convention a controller reports orientation in.
[[nodiscard]] std::array<double, 3> euler_deg(const cavr::core::Quaternion& q) {
  const double x = q.x(), y = q.y(), z = q.z(), w = q.w();
  const double roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
  const double sinp = std::clamp(2.0 * (w * y - z * x), -1.0, 1.0);
  const double pitch = std::asin(sinp);
  const double yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  return {roll * kRadToDeg, pitch * kRadToDeg, yaw * kRadToDeg};
}

// A stable, DB-friendly id from a display name (defined below; used by both the
// robot registry and the program store).
std::string slugify(const QString& name);
}  // namespace

RobotController::RobotController(QObject* parent) : QObject(parent) {
  // Open the robot registry (SQLite) in the app's writable data directory, so saved
  // robots persist across runs. If it can't be opened, the app still works — it just
  // falls back to the in-code presets and can't save.
  const QString data_dir = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
  QDir().mkpath(data_dir);
  const std::string db_path = (data_dir + "/robot_registry.db").toStdString();
  registry_ = std::make_unique<cavr::catalog::SqliteProfileStore>(
      cavr::catalog::CatalogOpenOptions{db_path, true});
  if (auto status = registry_->initialize(); !status) {
    emit eventLogged(QString("registry | init failed: ") + QString::fromStdString(status.error));
    registry_.reset();
  }
  seed_default_robots();

  // Program (job) registry lives beside the robot registry.
  const std::string prog_path = (data_dir + "/program_registry.db").toStdString();
  programs_ = std::make_unique<cavr::catalog::SqliteProgramStore>(
      cavr::catalog::CatalogOpenOptions{prog_path, true});
  if (auto status = programs_->initialize(); !status) {
    emit eventLogged(QString("programs | init failed: ") + QString::fromStdString(status.error));
    programs_.reset();
  }

  connect(&timer_, &QTimer::timeout, this, &RobotController::tick);

  // Initial robot. CAVR_ROBOT_ENDPOINT=host:port still drives the scene from a
  // remote robot over TCP (back-compat); otherwise connect the first registered
  // robot, or a GP25 preset if the registry is unavailable.
  if (const char* endpoint = std::getenv("CAVR_ROBOT_ENDPOINT"); endpoint && *endpoint) {
    connectRobot("generic_tcp", endpoint, cavr::adapters::mock_robot::make_gp25_profile());
  } else if (registry_) {
    const auto robots = registry_->list_robots();
    if (!robots.empty())
      connectRobot(robots.front().adapter, robots.front().endpoint, robots.front().profile);
    else
      connectRobot("mock", "", cavr::adapters::mock_robot::make_gp25_profile());
  } else {
    connectRobot("mock", "", cavr::adapters::mock_robot::make_gp25_profile());
  }
  timer_.start(20);
}

void RobotController::connectRobot(const std::string& adapter, const std::string& endpoint,
                                   const cavr::machine::MachineProfile& profile) {
  remote_ = (adapter != "mock");
  controller_ = cavr::studio::make_adapter(adapter, profile);

  cavr::adapter_sdk::ConnectionInfo info{endpoint.empty() ? std::string("mock") : endpoint,
                                         remote_ ? "tcp" : "mock"};
  // connect -> discover profile -> plan -> validate. discover_profile() returns the
  // mock's injected profile or the TCP bridge's, so the scene mirrors whichever
  // robot was chosen through the one ControllerAdapter seam. The demo is NOT started
  // here: the robot holds its home pose on connect and only moves when the operator
  // presses Run Demo or jogs it. (A remote robot drives its own motion; we mirror.)
  static_cast<void>(manager_.connect(*controller_, info));
  static_cast<void>(manager_.discover_profile());
  manager_.set_plan(cavr::runtime::make_demo_plan());
  static_cast<void>(manager_.validate());
  if (remote_) {
    // A remote robot is already running in the field; start mirroring its stream.
    static_cast<void>(manager_.execute("studio_session_" + std::to_string(run_index_)));
  } else {
    // A local mock is a simulation: hold the home pose and only move on Run Demo/jog.
    manual_ = true;
  }
  publish();
  emit robotsChanged();
}

void RobotController::seed_default_robots() {
  if (!registry_ || !registry_->list_robots().empty()) return;

  cavr::catalog::StoredRobot gp;
  gp.id = "gp25_cell1";
  gp.display_name = "Yaskawa GP25 (mock)";
  gp.profile = cavr::adapters::mock_robot::make_gp25_profile();
  gp.adapter = "mock";
  gp.transport = "mock";
  static_cast<void>(registry_->upsert_robot(gp));

  cavr::catalog::StoredRobot pnr;
  pnr.id = "pnr_cell1";
  pnr.display_name = "PNR 6-Axis (mock)";
  pnr.profile = cavr::adapters::mock_robot::make_pnr_profile();
  pnr.adapter = "mock";
  pnr.transport = "mock";
  static_cast<void>(registry_->upsert_robot(pnr));
}

void RobotController::tick() {
  now_ns_ += kTickNs;
  manager_.tick(cavr::core::Timestamp::from_nanoseconds(now_ns_));

  for (const auto& e : manager_.latest().events) {
    emit eventLogged(QString::fromStdString(cavr::machine::to_string(e.kind) + " | " + e.message));
  }

  // For the in-process mock, loop the demo so the cell keeps running like a
  // repeating production cycle — unless the operator has taken manual control by
  // jogging, in which case the robot holds its jogged pose. A remote robot drives
  // its own motion (cavr-robotd loops continuously), so we just mirror it.
  if (!remote_ && !manual_ && manager_.phase() == cavr::runtime::SessionPhase::Completed) {
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

  // TCP readout in the selected coordinate system (mm + degrees). The telemetry
  // reports the TCP in the base frame; User is expressed relative to the User
  // frame, the others coincide with base at the world origin.
  cavr::core::Pose3D shown = s.tcp_pose;
  if (coord_sys_ == cavr::machine::CoordinateSystem::User) {
    cavr::core::Pose3D user_frame;
    for (const auto& f : manager_.profile().frames) {
      if (f.kind == cavr::machine::FrameKind::User) { user_frame = f.transform; break; }
    }
    shown = cavr::machine::compose(cavr::machine::invert(user_frame), s.tcp_pose);
  }
  const std::array<double, 3> rpy = euler_deg(shown.orientation);
  tcp_coords_ = QVariantList{shown.position_m.x_m * 1000.0, shown.position_m.y_m * 1000.0,
                             shown.position_m.z_m * 1000.0, rpy[0], rpy[1], rpy[2]};
  {
    QString name = QString::fromStdString(cavr::machine::to_string(coord_sys_));
    if (!name.isEmpty()) name[0] = name[0].toUpper();
    coord_system_name_ = name;
  }

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

void RobotController::jogHome() {
  cavr::machine::MotionCommand cmd;
  cmd.kind = cavr::machine::MotionKind::MoveJ;
  const std::size_t dof = manager_.profile().axes.size();
  cmd.target.joints = std::vector<double>(dof > 0 ? dof : 6, 0.0);  // all axes to home
  cmd.speed = 45.0 * 3.14159265358979323846 / 180.0;                // 45 deg/s
  cmd.label = "jog home";
  manual_ = true;
  running_current_program_ = false;
  static_cast<void>(controller_->move_to(cmd));
  publish();
}

void RobotController::jogJoint(int axis, double delta_deg) {
  const auto& latest = manager_.latest();
  std::vector<double> target = latest.joint_positions;
  const std::size_t dof = manager_.profile().axes.size();
  target.resize(dof > 0 ? dof : 6, 0.0);
  if (axis < 0 || axis >= static_cast<int>(target.size())) return;
  target[static_cast<std::size_t>(axis)] += delta_deg * 3.14159265358979323846 / 180.0;

  cavr::machine::MotionCommand cmd;
  cmd.kind = cavr::machine::MotionKind::MoveJ;
  cmd.target.joints = std::move(target);
  cmd.speed = 45.0 * 3.14159265358979323846 / 180.0;
  cmd.label = "jog joint";
  manual_ = true;
  running_current_program_ = false;
  static_cast<void>(controller_->move_to(cmd));
  publish();
}

void RobotController::jogCartesian(double dx_m, double dy_m, double dz_m,
                                   double drx_rad, double dry_rad, double drz_rad) {
  // Move the TCP by the given delta expressed in the selected coordinate system,
  // then let the controller solve IK for the resulting pose (MoveL-style jog).
  const auto& latest = manager_.latest();

  // The User frame, when selected, comes from the profile's first User frame.
  cavr::core::Pose3D user_frame;
  for (const auto& f : manager_.profile().frames) {
    if (f.kind == cavr::machine::FrameKind::User) { user_frame = f.transform; break; }
  }

  const cavr::core::Pose3D target = cavr::machine::jog_in_frame(
      latest.tcp_pose, coord_sys_, {dx_m, dy_m, dz_m}, {drx_rad, dry_rad, drz_rad}, user_frame);

  cavr::machine::MotionCommand cmd;
  cmd.kind = cavr::machine::MotionKind::MoveL;
  cmd.target.pose = target;
  cmd.speed = speed_mm_s_;  // mm/s
  cmd.label = "jog cartesian";
  manual_ = true;
  running_current_program_ = false;
  if (!controller_->move_to(cmd)) {
    emit eventLogged("jog cartesian | target unreachable (IK did not converge)");
  }
  publish();
}

void RobotController::jogArc() {
  // A visible MoveC demo: arc from the current TCP through a via offset up-and-out
  // to an end offset further along, curving through 3 non-collinear points.
  const auto& latest = manager_.latest();
  cavr::core::Pose3D via = latest.tcp_pose;
  via.position_m.x_m += 0.05;
  via.position_m.z_m += 0.05;
  cavr::core::Pose3D end = latest.tcp_pose;
  end.position_m.x_m += 0.10;

  cavr::machine::MotionCommand cmd;
  cmd.kind = cavr::machine::MotionKind::MoveC;
  cmd.via = via;
  cmd.target.pose = end;
  cmd.speed = speed_mm_s_;  // mm/s
  cmd.label = "jog arc (MoveC)";
  manual_ = true;
  running_current_program_ = false;
  if (!controller_->move_to(cmd)) {
    emit eventLogged("jog arc | target unreachable (IK did not converge)");
  }
  publish();
}

void RobotController::setCoordinateSystem(int system) {
  switch (system) {
    case 0: coord_sys_ = cavr::machine::CoordinateSystem::World; break;
    case 2: coord_sys_ = cavr::machine::CoordinateSystem::Tool; break;
    case 3: coord_sys_ = cavr::machine::CoordinateSystem::User; break;
    case 1:
    default: coord_sys_ = cavr::machine::CoordinateSystem::Base; break;
  }
  emit eventLogged(QString("coordinate system | ") + cavr::machine::to_string(coord_sys_));
  publish();  // refresh the TCP readout in the newly selected frame
}

void RobotController::setSpeedMmS(double mm_s) {
  if (mm_s > 0.0) speed_mm_s_ = mm_s;
}

void RobotController::selectTool(int slot) {
  // Goes through the adapter so a remote controller is kept in sync (the mock
  // mutates its table directly; the TCP controller sends a protocol command).
  if (controller_->select_tool(slot)) {
    emit eventLogged(QString("tool | selected slot %1").arg(slot));
    publish();
  }
}

void RobotController::calibrateTool(int slot, double x_m, double y_m, double z_m) {
  const cavr::core::Pose3D tcp{cavr::core::Vec3{x_m, y_m, z_m}, cavr::core::Quaternion::identity()};
  if (controller_->calibrate_tool(slot, tcp)) {
    emit eventLogged(QString("tool | calibrated slot %1 TCP (%2, %3, %4) m")
                         .arg(slot).arg(x_m).arg(y_m).arg(z_m));
    publish();
  }
}

void RobotController::clearTool(int slot) {
  if (controller_->clear_tool(slot)) {
    emit eventLogged(QString("tool | cleared slot %1").arg(slot));
    publish();
  }
}

void RobotController::runDemo() {
  manual_ = false;
  running_current_program_ = false;
  manager_.set_plan(cavr::runtime::make_demo_plan());
  static_cast<void>(manager_.validate());
  static_cast<void>(manager_.execute("studio_session_" + std::to_string(++run_index_)));
  publish();
}
void RobotController::stop() {
  manager_.stop();
  publish();
}

bool RobotController::saveSession(const QString& path) {
  std::vector<std::string> errors;
  return cavr::runtime::save_session_log(manager_.log(), path.toStdString(), errors);
}

namespace {
// A short human summary of a step for the editor list.
QString step_detail(const cavr::machine::MotionCommand& c) {
  using cavr::machine::MotionKind;
  switch (c.kind) {
    case MotionKind::MoveJ:
      return c.target.joints ? QString("%1 axes").arg(c.target.joints->size()) : "—";
    case MotionKind::MoveL:
    case MotionKind::MoveC: {
      if (!c.target.pose) return "—";
      const auto& p = c.target.pose->position_m;
      QString s = QString("X%1 Y%2 Z%3")
                      .arg(p.x_m * 1000.0, 0, 'f', 0)
                      .arg(p.y_m * 1000.0, 0, 'f', 0)
                      .arg(p.z_m * 1000.0, 0, 'f', 0);
      if (c.kind == MotionKind::MoveC) s += " (arc)";
      return s;
    }
    case MotionKind::Wait:
      return QString("%1 s").arg(c.wait_s, 0, 'f', 1);
    case MotionKind::ToolOn:
    case MotionKind::ToolOff:
      return c.label.empty() ? QString() : QString::fromStdString(c.label);
  }
  return {};
}
}  // namespace

void RobotController::addMoveL() {
  cavr::machine::MotionCommand ml;
  ml.kind = cavr::machine::MotionKind::MoveL;
  ml.target.pose = manager_.latest().tcp_pose;
  ml.speed = speed_mm_s_;
  ml.label = "MoveL";
  program_.append(std::move(ml));
  running_current_program_ = false;
  emit eventLogged("program | added MoveL (current pose)");
  emit programSelectionChanged();
  emit programChanged();
}

void RobotController::addMoveJ() {
  cavr::machine::MotionCommand mj;
  mj.kind = cavr::machine::MotionKind::MoveJ;
  mj.target.joints = manager_.latest().joint_positions;
  mj.speed = 45.0 * 3.14159265358979323846 / 180.0;  // rad/s
  mj.label = "MoveJ";
  program_.append(std::move(mj));
  running_current_program_ = false;
  emit eventLogged("program | added MoveJ (current joints)");
  emit programSelectionChanged();
  emit programChanged();
}

void RobotController::setVia() {
  program_.set_pending_via(manager_.latest().tcp_pose);
  emit eventLogged("program | via point stashed (next MoveC will arc through it)");
}

void RobotController::addMoveC() {
  auto via = program_.take_pending_via();
  if (!via) {
    emit eventLogged("program | set a via first (Set via), then Add MoveC");
    return;
  }
  cavr::machine::MotionCommand mc;
  mc.kind = cavr::machine::MotionKind::MoveC;
  mc.via = *via;
  mc.target.pose = manager_.latest().tcp_pose;
  mc.speed = speed_mm_s_;
  mc.label = "MoveC";
  program_.append(std::move(mc));
  running_current_program_ = false;
  emit eventLogged("program | added MoveC (arc via stashed point)");
  emit programSelectionChanged();
  emit programChanged();
}

void RobotController::addWait(double seconds) {
  cavr::machine::MotionCommand w;
  w.kind = cavr::machine::MotionKind::Wait;
  w.wait_s = seconds > 0 ? seconds : 0.0;
  w.label = "Wait";
  program_.append(std::move(w));
  running_current_program_ = false;
  emit eventLogged(QString("program | added Wait %1 s").arg(seconds));
  emit programSelectionChanged();
  emit programChanged();
}

void RobotController::addIoSet(const QString& channel, double value) {
  // Modeled as a ToolOn/ToolOff-style step carrying the IO in its label; on run we
  // apply it via write_io at that step. For simplicity we set it immediately and
  // record a labelled Wait(0) marker so the program lists the action.
  static_cast<void>(controller_->write_io(channel.toStdString(), value));
  cavr::machine::MotionCommand io;
  io.kind = cavr::machine::MotionKind::Wait;
  io.wait_s = 0.0;
  io.label = "IO " + channel.toStdString() + "=" + std::to_string(value);
  program_.append(std::move(io));
  running_current_program_ = false;
  emit eventLogged(QString("program | added IO set %1=%2").arg(channel).arg(value));
  emit programSelectionChanged();
  emit programChanged();
}

void RobotController::addToolOn() {
  cavr::machine::MotionCommand c;
  c.kind = cavr::machine::MotionKind::ToolOn;
  c.label = "ToolOn";
  program_.append(std::move(c));
  running_current_program_ = false;
  emit eventLogged("program | added ToolOn");
  emit programSelectionChanged();
  emit programChanged();
}

void RobotController::addToolOff() {
  cavr::machine::MotionCommand c;
  c.kind = cavr::machine::MotionKind::ToolOff;
  c.label = "ToolOff";
  program_.append(std::move(c));
  running_current_program_ = false;
  emit eventLogged("program | added ToolOff");
  emit programSelectionChanged();
  emit programChanged();
}

void RobotController::removeStep(int index) {
  if (!program_.remove_step(index)) return;
  emit programSelectionChanged();
  emit programChanged();
}

void RobotController::moveStep(int index, int delta) {
  if (!program_.move_step(index, delta)) return;
  emit programSelectionChanged();
  emit programChanged();
}

void RobotController::clearProgram() {
  program_.clear();
  running_current_program_ = false;
  emit eventLogged("program | cleared");
  emit programSelectionChanged();
  emit programChanged();
}

void RobotController::runProgram() {
  if (program_.empty()) {
    emit eventLogged("program | empty, nothing to run");
    return;
  }

  manual_ = false;
  running_current_program_ = true;
  manager_.set_plan(program_.to_timeline());
  static_cast<void>(manager_.validate());
  static_cast<void>(manager_.execute("studio_program_" + std::to_string(++run_index_)));
  emit eventLogged(QString("program | running %1 steps").arg(program_.size()));
  publish();
}

QVariantList RobotController::programSteps() const {
  QVariantList out;
  const int active = running_current_program_ ? manager_.latest().current_step : -1;
  const cavr::machine::MotionTask& task = program_.task();
  for (std::size_t i = 0; i < task.size(); ++i) {
    const auto& c = task[i];
    QVariantMap m;
    m["index"] = static_cast<int>(i + 1);
    m["zeroIndex"] = static_cast<int>(i);
    m["kind"] = QString::fromStdString(cavr::machine::to_string(c.kind));
    m["detail"] = step_detail(c);
    m["label"] = QString::fromStdString(c.label);
    m["speed"] = c.speed;
    m["blend"] = c.blend_radius_m;
    m["wait"] = c.wait_s;
    m["selected"] = program_.selected_step() == static_cast<int>(i);
    m["active"] = active == static_cast<int>(i);
    m["duration"] = c.kind == cavr::machine::MotionKind::Wait
                        ? std::max(0.15, c.wait_s)
                        : (c.kind == cavr::machine::MotionKind::ToolOn ||
                           c.kind == cavr::machine::MotionKind::ToolOff)
                              ? 0.25
                              : 1.0;
    out.push_back(m);
  }
  return out;
}

void RobotController::selectProgramStep(int index) {
  const int before = program_.selected_step();
  program_.select_step(index);
  if (program_.selected_step() == before) return;
  emit programSelectionChanged();
  emit programChanged();
}

QVariantList RobotController::savedPrograms() const {
  QVariantList out;
  if (!programs_) return out;
  for (const auto& p : programs_->list_programs()) {
    QVariantMap m;
    m["id"] = QString::fromStdString(p.id);
    m["name"] = QString::fromStdString(p.name);
    m["robot"] = QString::fromStdString(p.robot_id);
    m["steps"] = static_cast<int>(p.task.size());
    out.push_back(m);
  }
  return out;
}

void RobotController::saveProgram(const QString& name) {
  if (!programs_) {
    emit eventLogged("programs | store unavailable");
    return;
  }
  if (program_.empty()) {
    emit eventLogged("programs | nothing to save (empty program)");
    return;
  }
  cavr::catalog::StoredProgram p;
  // Reuse the robot slug helper for a stable id.
  p.id = slugify(name);
  p.name = name.trimmed().isEmpty() ? std::string("Unnamed job") : name.toStdString();
  p.robot_id = manager_.profile().id;
  p.task = program_.task();
  p.updated_ns = now_ns_;
  if (auto status = programs_->upsert_program(p); !status) {
    emit eventLogged(QString("programs | save failed: ") + QString::fromStdString(status.error));
    return;
  }
  emit eventLogged("programs | saved " + QString::fromStdString(p.name) + " (" +
                   QString::fromStdString(p.id) + ")");
  emit savedProgramsChanged();
}

void RobotController::loadProgram(const QString& id) {
  if (!programs_) return;
  const auto p = programs_->find_program(id.toStdString());
  if (!p) {
    emit eventLogged("programs | unknown program: " + id);
    return;
  }
  program_.set_task(p->task);
  emit eventLogged("programs | loaded " + QString::fromStdString(p->name));
  emit programSelectionChanged();
  emit programChanged();
}

void RobotController::deleteProgram(const QString& id) {
  if (!programs_) return;
  if (auto status = programs_->delete_program(id.toStdString()); !status) {
    emit eventLogged(QString("programs | delete failed: ") + QString::fromStdString(status.error));
    return;
  }
  emit eventLogged("programs | deleted " + id);
  emit savedProgramsChanged();
}

namespace {
// A stable, filesystem/DB-friendly id from a display name: lowercase alphanumerics,
// runs of anything else collapsed to a single '_'. Falls back to "robot".
std::string slugify(const QString& name) {
  std::string out;
  bool last_us = false;
  for (const QChar qc : name) {
    const char c = static_cast<char>(qc.toLatin1());
    if (std::isalnum(static_cast<unsigned char>(c))) {
      out.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
      last_us = false;
    } else if (!out.empty() && !last_us) {
      out.push_back('_');
      last_us = true;
    }
  }
  while (!out.empty() && out.back() == '_') out.pop_back();
  return out.empty() ? std::string("robot") : out;
}
}  // namespace

QVariantList RobotController::robotList() const {
  QVariantList out;
  if (!registry_) return out;
  for (const auto& r : registry_->list_robots()) {
    QVariantMap m;
    m["id"] = QString::fromStdString(r.id);
    m["name"] = QString::fromStdString(r.display_name);
    m["model"] = QString::fromStdString(r.profile.robot_model);
    m["adapter"] = QString::fromStdString(r.adapter);
    m["endpoint"] = QString::fromStdString(r.endpoint);
    m["dof"] = static_cast<int>(r.profile.dof());
    m["ioCount"] = static_cast<int>(r.profile.io.size());
    out.push_back(m);
  }
  return out;
}

void RobotController::saveRobot(const QString& name, const QString& adapter,
                                const QString& endpoint) {
  if (!registry_) {
    emit eventLogged("registry | unavailable, cannot save");
    return;
  }
  cavr::catalog::StoredRobot r;
  r.id = slugify(name);
  r.display_name = name.trimmed().isEmpty() ? std::string("Unnamed robot") : name.toStdString();
  r.profile = manager_.profile();  // the currently connected robot's profile
  r.adapter = adapter.isEmpty() ? std::string("mock") : adapter.toStdString();
  r.transport = (r.adapter == "mock") ? "mock" : "tcp";
  r.endpoint = endpoint.toStdString();
  r.updated_ns = now_ns_;
  if (auto status = registry_->upsert_robot(r); !status) {
    emit eventLogged(QString("registry | save failed: ") + QString::fromStdString(status.error));
    return;
  }
  emit eventLogged("registry | saved " + QString::fromStdString(r.display_name) + " (" +
                   QString::fromStdString(r.id) + ")");
  emit robotsChanged();
}

void RobotController::loadRobot(const QString& id) {
  if (!registry_) return;
  const auto r = registry_->find_robot(id.toStdString());
  if (!r) {
    emit eventLogged("registry | unknown robot: " + id);
    return;
  }
  ++run_index_;
  connectRobot(r->adapter, r->endpoint, r->profile);
  emit eventLogged("registry | loaded " + QString::fromStdString(r->display_name));
}

void RobotController::deleteRobot(const QString& id) {
  if (!registry_) return;
  if (auto status = registry_->delete_robot(id.toStdString()); !status) {
    emit eventLogged(QString("registry | delete failed: ") + QString::fromStdString(status.error));
    return;
  }
  emit eventLogged("registry | deleted " + id);
  emit robotsChanged();
}

QVariantList RobotController::ioChannels() const {
  // Live values come from the latest telemetry frame, keyed by channel name.
  const auto& latest = manager_.latest();
  QVariantList out;
  for (const auto& c : manager_.profile().io) {
    double value = 0.0;
    for (const auto& io : latest.io) {
      if (io.name == c.name) { value = io.value; break; }
    }
    QVariantMap m;
    m["name"] = QString::fromStdString(c.name);
    m["kind"] = QString::fromStdString(cavr::machine::to_string(c.kind));
    m["direction"] = QString::fromStdString(cavr::machine::to_string(c.direction));
    m["variable"] = QString::fromStdString(c.controller_variable);
    m["value"] = value;
    m["writable"] = c.direction != cavr::machine::IoDirection::Input;
    out.push_back(m);
  }
  return out;
}

void RobotController::writeIo(const QString& name, double value) {
  if (!controller_->write_io(name.toStdString(), value)) {
    emit eventLogged("io | write rejected: " + name);
    return;
  }
  emit eventLogged(QString("io | %1 = %2").arg(name).arg(value));
  publish();
}
