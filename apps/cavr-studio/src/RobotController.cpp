#include "RobotController.hpp"

#include "AdapterFactory.hpp"

#include <cavr/machine/enums.hpp>
#include <cavr/machine/ik.hpp>
#include <cavr/runtime/demo_plan.hpp>
#include <cavr/runtime/session_io.hpp>
#include <cavr/validation/trajectory_validator.hpp>

#include <QDir>
#include <QFileInfo>
#include <QStandardPaths>
#include <QVariantMap>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
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

  // Attach a synchronized vision source: while a session runs, the manager polls a
  // scan PointCloud on the same tick as the robot, so the vision-guided seam
  // correction has live 3D data.
  manager_.attach_camera(camera_);

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
  // Replay drives the scene from the loaded recording, not the live robot. When
  // playing, advance the playhead by one tick and stop at the end.
  if (replaying_) {
    if (replay_playing_) {
      const double dur = cavr::runtime::ReplayCursor(replay_log_).duration_s();
      replay_pos_s_ += static_cast<double>(kTickNs) * 1e-9;
      if (replay_pos_s_ >= dur) { replay_pos_s_ = dur; replay_playing_ = false; }
      publish();
      emit replayChanged();
    }
    return;
  }

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

const cavr::adapter_sdk::RobotState& RobotController::active_frame() const {
  if (replaying_) {
    if (const auto* f = cavr::runtime::ReplayCursor(replay_log_).frame_at(replay_pos_s_)) return *f;
  }
  return manager_.latest();
}

const cavr::machine::MachineProfile& RobotController::active_profile() const {
  return replaying_ ? replay_log_.profile : manager_.profile();
}

void RobotController::publish() {
  const auto& s = active_frame();

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
    for (const auto& f : active_profile().frames) {
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

  phase_ = replaying_ ? QStringLiteral("replay")
                      : QString::fromStdString(cavr::runtime::to_string(manager_.phase()));
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
  const auto& axes = manager_.profile().axes;
  std::vector<double> target = latest.joint_positions;
  const std::size_t dof = axes.size();
  target.resize(dof > 0 ? dof : 6, 0.0);
  if (axis < 0 || axis >= static_cast<int>(target.size())) return;

  double desired = target[static_cast<std::size_t>(axis)] + delta_deg * 3.14159265358979323846 / 180.0;
  // Enforce the axis's configured joint limits: clamp and report rather than driving
  // the robot past its mechanical range.
  if (axis < static_cast<int>(axes.size())) {
    const auto& a = axes[static_cast<std::size_t>(axis)];
    if (desired < a.lower_limit || desired > a.upper_limit) {
      desired = std::clamp(desired, a.lower_limit, a.upper_limit);
      emit eventLogged(QString("jog | axis %1 at joint limit — move clamped")
                           .arg(QString::fromStdString(a.name)));
    }
  }
  target[static_cast<std::size_t>(axis)] = desired;

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

bool RobotController::loadReplay(const QString& path) {
  auto result = cavr::runtime::load_session_log(path.toStdString());
  if (!result.ok() || result.log.frames.empty()) {
    const QString why = result.errors.empty() ? QStringLiteral("no frames")
                                              : QString::fromStdString(result.errors.front());
    emit eventLogged(QString("replay | could not load %1: %2").arg(QFileInfo(path).fileName(), why));
    return false;
  }
  replay_log_ = std::move(result.log);
  replay_file_ = QFileInfo(path).fileName();
  replaying_ = true;
  replay_playing_ = false;
  replay_pos_s_ = 0.0;
  emit eventLogged(QString("replay | loaded %1 (%2 frames)")
                       .arg(replay_file_)
                       .arg(replay_log_.frames.size()));
  publish();
  emit replayChanged();
  return true;
}

void RobotController::replaySeek(double fraction) {
  if (!replaying_) return;
  const double dur = cavr::runtime::ReplayCursor(replay_log_).duration_s();
  replay_pos_s_ = std::clamp(fraction, 0.0, 1.0) * dur;
  replay_playing_ = false;
  publish();
  emit replayChanged();
}

void RobotController::replayPlayPause() {
  if (!replaying_) return;
  const double dur = cavr::runtime::ReplayCursor(replay_log_).duration_s();
  if (replay_pos_s_ >= dur) replay_pos_s_ = 0.0;  // restart from the top if at the end
  replay_playing_ = !replay_playing_;
  emit replayChanged();
}

void RobotController::exitReplay() {
  if (!replaying_) return;
  replaying_ = false;
  replay_playing_ = false;
  emit eventLogged("replay | returned to live robot");
  publish();
  emit replayChanged();
}

double RobotController::replayPositionFraction() const {
  if (!replaying_) return 0.0;
  const double dur = cavr::runtime::ReplayCursor(replay_log_).duration_s();
  return dur > 0.0 ? std::clamp(replay_pos_s_ / dur, 0.0, 1.0) : 0.0;
}

QString RobotController::replayInfo() const {
  if (!replaying_) return "live";
  const cavr::runtime::ReplayCursor cursor(replay_log_);
  const double dur = cursor.duration_s();
  // Which frame the playhead is on (for the read-out).
  int index = 0;
  const std::int64_t base = replay_log_.frames.front().timestamp.nanoseconds();
  const std::int64_t want = base + static_cast<std::int64_t>(replay_pos_s_ * 1e9);
  for (std::size_t i = 0; i < replay_log_.frames.size(); ++i) {
    if (replay_log_.frames[i].timestamp.nanoseconds() <= want) index = static_cast<int>(i);
    else break;
  }
  return QString("%1  •  frame %2/%3  •  %4 / %5 s")
      .arg(replay_file_)
      .arg(index + 1)
      .arg(replay_log_.frames.size())
      .arg(replay_pos_s_, 0, 'f', 2)
      .arg(dur, 0, 'f', 2);
}

// ------------------------------------------------------------ calibration

void RobotController::captureCalibrationSample() {
  const auto& profile = active_profile();
  // The "true" hand-eye (camera-in-flange) we recover: the profile's camera
  // extrinsics. A fixed calibration target sits on the table in the base frame.
  const cavr::core::Pose3D true_he =
      profile.cameras.empty() ? cavr::core::Pose3D{} : profile.cameras.front().extrinsics;
  const cavr::core::Pose3D target_in_base{cavr::core::Vec3{0.6, 0.4, 0.0},
                                          cavr::core::Quaternion::identity()};

  const auto fk = cavr::machine::forward_kinematics(profile.axes, active_frame().joint_positions,
                                                    cavr::core::Pose3D{});
  const cavr::core::Pose3D flange = fk.tcp;  // identity tool → flange frame
  const cavr::core::Pose3D camera_in_base = cavr::machine::compose(flange, true_he);

  cavr::calibration::HandEyeSample sample;
  sample.flange_in_base = flange;
  // What the camera detector would report: the target expressed in the camera frame.
  sample.target_in_camera =
      cavr::machine::compose(cavr::machine::invert(camera_in_base),
                             cavr::core::Pose3D{target_in_base.position_m, target_in_base.orientation});
  he_samples_.push_back(sample);
  he_solved_ = false;
  emit eventLogged(QString("calibration | captured pose %1 (jog to vary, need ≥3)")
                       .arg(he_samples_.size()));
  emit calibrationChanged();
}

void RobotController::solveHandEye() {
  if (he_samples_.size() < 3) {
    emit eventLogged("calibration | need at least 3 varied poses before solving");
    return;
  }
  const std::string cam =
      active_profile().cameras.empty() ? std::string("camera") : active_profile().cameras.front().name;
  const auto result = cavr::calibration::solve_hand_eye(he_samples_,
                                                        cavr::calibration::HandEyeType::EyeInHand, cam);
  if (!result.ok) {
    he_solved_ = false;
    emit eventLogged(QString("calibration | solve failed: %1 — jog to more varied orientations")
                         .arg(QString::fromStdString(result.error)));
  } else {
    solved_he_ = result.calibration;
    he_solved_ = true;
    emit eventLogged(QString("calibration | solved from %1 poses — residual %2 mm / %3°")
                         .arg(result.pairs_used + 1)
                         .arg(solved_he_.residual_position_m * 1000.0, 0, 'f', 2)
                         .arg(solved_he_.residual_rotation_rad * 180.0 / 3.14159265358979323846, 0, 'f', 2));
  }
  emit calibrationChanged();
}

void RobotController::clearCalibrationSamples() {
  he_samples_.clear();
  he_solved_ = false;
  emit eventLogged("calibration | cleared samples");
  emit calibrationChanged();
}

QString RobotController::intrinsicsSummary() const {
  return QString("fx %1  fy %2  ·  cx %3  cy %4  ·  %5×%6")
      .arg(intrinsics_.fx, 0, 'f', 0)
      .arg(intrinsics_.fy, 0, 'f', 0)
      .arg(intrinsics_.cx, 0, 'f', 0)
      .arg(intrinsics_.cy, 0, 'f', 0)
      .arg(intrinsics_.width)
      .arg(intrinsics_.height);
}

QString RobotController::handEyeSummary() const {
  if (!he_solved_) return "not calibrated";
  const auto& t = solved_he_.transform.position_m;
  return QString("t (%1, %2, %3) mm  ·  residual %4 mm / %5°")
      .arg(t.x_m * 1000.0, 0, 'f', 1)
      .arg(t.y_m * 1000.0, 0, 'f', 1)
      .arg(t.z_m * 1000.0, 0, 'f', 1)
      .arg(solved_he_.residual_position_m * 1000.0, 0, 'f', 2)
      .arg(solved_he_.residual_rotation_rad * 180.0 / 3.14159265358979323846, 0, 'f', 2);
}

QString RobotController::calibrationStatus() const {
  if (he_solved_) return QString("calibrated (%1 poses)").arg(he_samples_.size());
  return QString("%1 pose%2 captured — need ≥3, then Solve")
      .arg(he_samples_.size())
      .arg(he_samples_.size() == 1 ? "" : "s");
}

bool RobotController::saveCalibration(const QString& path) {
  cavr::json::Value root;
  root.set("intrinsics", cavr::calibration::to_json(intrinsics_));
  if (he_solved_) root.set("hand_eye", cavr::calibration::to_json(solved_he_));
  std::ofstream out(path.toStdString());
  if (!out) {
    emit eventLogged(QString("calibration | could not write %1").arg(QFileInfo(path).fileName()));
    return false;
  }
  out << root.dump(2) << '\n';
  emit eventLogged(QString("calibration | saved %1").arg(QFileInfo(path).fileName()));
  return true;
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

// Pre-flight report: the library checks (limits/frames/target) plus a reachability
// pass that IK-solves each Cartesian target along the chain (mirroring how the
// controller executes it), so unreachable MoveL/MoveC poses are caught before Run.
cavr::validation::ValidationReport full_report(const cavr::machine::MachineProfile& profile,
                                               const cavr::machine::MotionTask& task,
                                               const cavr::core::Pose3D& tool) {
  using namespace cavr::machine;
  // The base checks: axis limits, commanded speed, referenced frames, reachability
  // and the cycle-time estimate, all from the same planner the controller runs.
  cavr::validation::ValidationReport report = cavr::validation::validate_task(profile, task);

  std::vector<double> seed(profile.dof(), 0.0);
  for (std::size_t s = 0; s < task.size(); ++s) {
    const auto& c = task[s];
    if (c.kind == MotionKind::MoveJ && c.target.joints) {
      seed = *c.target.joints;
      seed.resize(profile.dof(), 0.0);
    } else if ((c.kind == MotionKind::MoveL || c.kind == MotionKind::MoveC) && c.target.pose) {
      const IkResult ik = inverse_kinematics(profile.axes, *c.target.pose, seed, tool);
      if (!ik.converged) {
        report.issues.push_back({Severity::Error, "Target pose unreachable (IK did not converge)",
                                 static_cast<int>(s)});
      } else {
        seed = ik.joints;
      }
    }
  }
  return report;
}

// Worst severity among issues for one step: 2=error, 1=warning, 0=ok.
int step_status(const cavr::validation::ValidationReport& report, int step) {
  int worst = 0;
  for (const auto& i : report.issues) {
    if (i.step_index != step) continue;
    if (i.severity == cavr::machine::Severity::Error || i.severity == cavr::machine::Severity::Critical)
      return 2;
    worst = std::max(worst, 1);
  }
  return worst;
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
  // Pre-flight: refuse to run a program with validation errors (unreachable poses,
  // limit violations). Warnings are allowed through.
  if (!programValid()) {
    emit eventLogged("program | BLOCKED — fix validation errors before running (" +
                     validationSummary() + ")");
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
  const cavr::core::Pose3D tool =
      controller_ && controller_->tools() ? controller_->tools()->current_offset() : cavr::core::Pose3D{};
  const auto report = full_report(manager_.profile(), task, tool);
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
    m["status"] = step_status(report, static_cast<int>(i));  // 0 ok, 1 warn, 2 error
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

QString RobotController::validationSummary() const {
  const cavr::machine::MotionTask& task = program_.task();
  if (task.empty()) return "no steps";
  const cavr::core::Pose3D tool =
      controller_ && controller_->tools() ? controller_->tools()->current_offset() : cavr::core::Pose3D{};
  const auto report = full_report(manager_.profile(), task, tool);
  const int errors = static_cast<int>(report.error_count());
  int warnings = 0;
  for (const auto& i : report.issues)
    if (i.severity == cavr::machine::Severity::Warning) ++warnings;

  if (errors == 0 && warnings == 0)
    return QString("✓ %1 steps valid").arg(task.size());
  QString s;
  if (errors > 0) s += QString("✗ %1 error%2").arg(errors).arg(errors == 1 ? "" : "s");
  if (warnings > 0) {
    if (!s.isEmpty()) s += ", ";
    s += QString("⚠ %1 warning%2").arg(warnings).arg(warnings == 1 ? "" : "s");
  }
  return s;
}

bool RobotController::programValid() const {
  const cavr::core::Pose3D tool =
      controller_ && controller_->tools() ? controller_->tools()->current_offset() : cavr::core::Pose3D{};
  return full_report(manager_.profile(), program_.task(), tool).ok();
}

QString RobotController::cycleSummary() const {
  const cavr::machine::MotionTask& task = program_.task();
  const char* mode = motion_profile_mode_ == 1 ? "S-curve" : "trapezoidal";
  if (task.empty()) return QString("— (%1)").arg(mode);
  const cavr::machine::MachineProfile& profile = manager_.profile();
  const cavr::core::Pose3D tool =
      controller_ && controller_->tools() ? controller_->tools()->current_offset() : cavr::core::Pose3D{};
  // Plan through the same libs/motion planner the executor uses, with the selected
  // velocity profile, so the figure is the wall-clock time the program will take.
  const cavr::motion::TaskPlan plan = cavr::motion::plan_task(
      profile.axes, tool, task, std::vector<double>(profile.dof(), 0.0), motion_limits_);
  return QString("~%1 s (%2)").arg(plan.cycle_time_s(), 0, 'f', 2).arg(mode);
}

void RobotController::setMotionProfile(int mode) {
  const int clamped = mode == 1 ? 1 : 0;
  if (clamped == motion_profile_mode_) return;
  motion_profile_mode_ = clamped;
  motion_limits_ = clamped == 1 ? cavr::motion::MotionLimits::s_curve()
                                : cavr::motion::MotionLimits::trapezoidal();
  // Retune the virtual robot so subsequent moves (and re-runs) execute under the new
  // profile. Only the in-process mock exposes the executor; a remote controller runs
  // its own profile, so we only update the cycle-time estimate there.
  if (auto* mock = dynamic_cast<cavr::adapters::mock_robot::MockController*>(controller_.get())) {
    mock->virtual_robot().set_limits(motion_limits_);
  }
  emit eventLogged(QString("motion | velocity profile → %1")
                       .arg(clamped == 1 ? "S-curve (jerk-limited)" : "trapezoidal"));
  emit programChanged();  // refresh the cycle-time read-out
}

namespace {
// The in-process virtual robot behind the mock adapter, or nullptr for a remote
// controller (which runs its own faults, out of the twin's reach).
cavr::sim::VirtualRobot* mock_robot(cavr::adapter_sdk::ControllerAdapter* c) {
  auto* mock = dynamic_cast<cavr::adapters::mock_robot::MockController*>(c);
  return mock ? &mock->virtual_robot() : nullptr;
}
}  // namespace

bool RobotController::faultSupported() const { return mock_robot(controller_.get()) != nullptr; }

void RobotController::injectEstop() {
  auto* robot = mock_robot(controller_.get());
  if (!robot) { emit eventLogged("fault | E-stop not supported on this controller"); return; }
  robot->faults().add(cavr::fault_injection::estop_at(0.0, "Operator emergency stop"));
  emit eventLogged("fault | E-stop injected — program will abort");
}

void RobotController::injectServoFault() {
  auto* robot = mock_robot(controller_.get());
  if (!robot) { emit eventLogged("fault | servo fault not supported on this controller"); return; }
  cavr::fault_injection::FaultSpec s;
  s.kind = cavr::fault_injection::FaultKind::ServoFault;
  s.trigger = cavr::fault_injection::TriggerKind::AtTime;
  s.at_time_s = 0.0;
  s.label = "Servo overload";
  robot->faults().add(s);
  emit eventLogged("fault | servo overload injected — program will abort");
}

void RobotController::injectArcLoss() {
  auto* robot = mock_robot(controller_.get());
  if (!robot) { emit eventLogged("fault | arc loss not supported on this controller"); return; }
  robot->faults().add(cavr::fault_injection::arc_loss_at(0.0, "Weld arc lost"));
  emit eventLogged("fault | arc loss injected — weld signals will drop");
}

void RobotController::setEncoderNoise(double stddev_deg) {
  auto* robot = mock_robot(controller_.get());
  encoder_noise_rad_ = std::max(0.0, stddev_deg) * 3.14159265358979323846 / 180.0;
  if (!robot) { emit eventLogged("fault | encoder noise not supported on this controller"); return; }
  // Rebuild the continuous specs. Discrete faults are one-shot and fire within a
  // tick, so re-arming the injector here does not lose a pending alarm in practice.
  robot->faults().clear();
  if (encoder_noise_rad_ > 0.0) robot->faults().add(cavr::fault_injection::encoder_noise(encoder_noise_rad_));
  robot->faults().arm();
  emit eventLogged(QString("fault | encoder noise → %1° std-dev").arg(stddev_deg, 0, 'f', 2));
}

void RobotController::resetFault() {
  auto* robot = mock_robot(controller_.get());
  if (!robot) { emit eventLogged("fault | nothing to reset on this controller"); return; }
  robot->clear_fault();
  emit eventLogged("fault | cleared — alarm acknowledged");
  publish();
}

QString RobotController::faultStatus() const {
  const auto& frame = active_frame();
  if (frame.faulted()) {
    return QString("⛔ %1 (code %2)")
        .arg(QString::fromStdString(frame.error_message.empty() ? "Faulted" : frame.error_message))
        .arg(frame.error_code);
  }
  if (frame.servo_state == cavr::machine::ServoState::Error) return "⛔ servo error";
  return "✓ no active fault";
}

bool RobotController::hasScan() const { return manager_.has_point_cloud(); }

int RobotController::scanPointCount() const {
  return static_cast<int>(manager_.latest_point_cloud().size());
}

// The seam correction (metres, base frame): place the live scan in the base frame
// via the profile's hand-eye extrinsics and the current flange pose, then compare
// its centroid to where the plan expects the seam (the program's first Cartesian
// target, else the current TCP).
cavr::core::Vec3 RobotController::seam_offset_m() const {
  if (!manager_.has_point_cloud()) return {};
  const auto& profile = manager_.profile();

  cavr::calibration::HandEyeCalibration he;
  he.type = cavr::calibration::HandEyeType::EyeInHand;
  if (!profile.cameras.empty()) {
    he.transform = profile.cameras.front().extrinsics;  // camera-in-flange
    he.camera_frame = profile.cameras.front().name;
  }

  const auto fk = cavr::machine::forward_kinematics(profile.axes, manager_.latest().joint_positions,
                                                    cavr::core::Pose3D{});
  const cavr::core::Pose3D flange = fk.tcp;  // identity tool offset → the flange frame

  const auto cloud_base = cavr::runtime::cloud_in_base(manager_.latest_point_cloud(), he, flange);

  cavr::core::Vec3 expected = manager_.latest().tcp_pose.position_m;
  for (const auto& c : program_.task()) {
    if ((c.kind == cavr::machine::MotionKind::MoveL || c.kind == cavr::machine::MotionKind::MoveC) &&
        c.target.pose) {
      expected = c.target.pose->position_m;
      break;
    }
  }
  return cavr::runtime::seam_offset(cloud_base, expected);
}

QVariantList RobotController::seamOffsetMm() const {
  if (!manager_.has_point_cloud()) return {};
  const cavr::core::Vec3 o = seam_offset_m();
  return QVariantList{o.x_m * 1000.0, o.y_m * 1000.0, o.z_m * 1000.0};
}

QString RobotController::visionSummary() const {
  if (!manager_.has_point_cloud()) return "no live scan — press Run Demo";
  const cavr::core::Vec3 o = seam_offset_m();
  return QString("scan %1 pts  •  Δ (%2, %3, %4) mm")
      .arg(scanPointCount())
      .arg(o.x_m * 1000.0, 0, 'f', 1)
      .arg(o.y_m * 1000.0, 0, 'f', 1)
      .arg(o.z_m * 1000.0, 0, 'f', 1);
}

void RobotController::applySeamCorrection() {
  if (!manager_.has_point_cloud()) {
    emit eventLogged("vision | no live scan — press Run Demo first");
    return;
  }
  if (program_.task().empty()) {
    emit eventLogged("vision | program is empty — nothing to correct");
    return;
  }
  const cavr::core::Vec3 o = seam_offset_m();
  program_.set_task(cavr::runtime::apply_seam_offset(program_.task(), o));
  running_current_program_ = false;
  emit eventLogged(QString("vision | applied seam correction Δ (%1, %2, %3) mm")
                       .arg(o.x_m * 1000.0, 0, 'f', 1)
                       .arg(o.y_m * 1000.0, 0, 'f', 1)
                       .arg(o.z_m * 1000.0, 0, 'f', 1));
  emit programSelectionChanged();
  emit programChanged();
}

QVariantList RobotController::jointStatus() const {
  QVariantList out;
  const auto& axes = active_profile().axes;
  const auto& q = active_frame().joint_positions;
  for (std::size_t i = 0; i < axes.size(); ++i) {
    const double value = (i < q.size() ? q[i] : 0.0) * kRadToDeg;
    const double lower = axes[i].lower_limit * kRadToDeg;
    const double upper = axes[i].upper_limit * kRadToDeg;
    QVariantMap m;
    m["name"] = QString::fromStdString(axes[i].name);
    m["value"] = value;
    m["lower"] = lower;
    m["upper"] = upper;
    m["over"] = value < lower - 1e-3 || value > upper + 1e-3;
    out.push_back(m);
  }
  return out;
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
