#pragma once

// Bridge between the runtime session layer and the Qt/QML UI. It drives the demo
// welding workflow through a SessionManager + mock controller and republishes the
// live telemetry (joint angles, program state, current step, weld, events) as Qt
// properties/signals so the 3D viewport and panels render real data, not an
// animation. Swapping the mock for a real ControllerAdapter changes nothing here.

#include <QObject>
#include <QString>
#include <QStringList>
#include <QTimer>
#include <QVariantList>

#include <cavr/adapter_sdk/controller_adapter.hpp>
#include <cavr/adapters/generic_tcp_robot/generic_tcp_controller.hpp>
#include <cavr/adapters/mock_robot/mock_controller.hpp>
#include <cavr/catalog/sqlite_profile_store.hpp>
#include <cavr/machine/frames.hpp>
#include <cavr/runtime/session_manager.hpp>

#include <cstdint>
#include <memory>
#include <string>

class RobotController final : public QObject {
  Q_OBJECT
  Q_PROPERTY(QVariantList jointDegrees READ jointDegrees NOTIFY telemetryChanged)
  Q_PROPERTY(QString phase READ phase NOTIFY telemetryChanged)
  Q_PROPERTY(QString programState READ programState NOTIFY telemetryChanged)
  Q_PROPERTY(QString stepLabel READ stepLabel NOTIFY telemetryChanged)
  Q_PROPERTY(double speedFraction READ speedFraction NOTIFY telemetryChanged)
  Q_PROPERTY(bool weldActive READ weldActive NOTIFY telemetryChanged)
  Q_PROPERTY(QVariantList tcpPosition READ tcpPosition NOTIFY telemetryChanged)

 public:
  explicit RobotController(QObject* parent = nullptr);

  [[nodiscard]] QVariantList jointDegrees() const { return joint_degrees_; }
  [[nodiscard]] QString phase() const { return phase_; }
  [[nodiscard]] QString programState() const { return program_state_; }
  [[nodiscard]] QString stepLabel() const { return step_label_; }
  [[nodiscard]] double speedFraction() const { return speed_fraction_; }
  [[nodiscard]] bool weldActive() const { return weld_active_; }
  [[nodiscard]] QVariantList tcpPosition() const { return tcp_position_; }

  Q_INVOKABLE void start();
  Q_INVOKABLE void pause();
  Q_INVOKABLE void resume();
  Q_INVOKABLE void stop();
  // Live jog from the scene (scene -> robot): command the robot — mock or remote —
  // to move now, interrupting the running program. Jogging switches the cell into
  // manual mode so it holds the jogged pose instead of resuming the demo.
  Q_INVOKABLE void jogHome();
  Q_INVOKABLE void jogJoint(int axis, double delta_deg);   // relative single-axis move
  // Relative Cartesian jog (metres / radians) in the selected coordinate system,
  // solved through IK. Translation and rotation components are independent.
  Q_INVOKABLE void jogCartesian(double dx_m, double dy_m, double dz_m,
                                double drx_rad, double dry_rad, double drz_rad);
  Q_INVOKABLE void setCoordinateSystem(int system);        // 0 World, 1 Base, 2 Tool, 3 User
  Q_INVOKABLE void setSpeedMmS(double mm_s);               // Cartesian jog speed
  Q_INVOKABLE void selectTool(int slot);                   // choose the active tool
  Q_INVOKABLE void calibrateTool(int slot, double x_m, double y_m, double z_m);  // set TCP offset
  Q_INVOKABLE void clearTool(int slot);
  Q_INVOKABLE void runDemo();                              // leave manual mode, resume the demo
  Q_INVOKABLE bool saveSession(const QString& path);

  // Robot registry (the universal-SDK seam): list saved robots, save the currently
  // connected one under a name, load a saved robot (reconnecting through its
  // adapter), or delete one. Each entry is a map {id, name, model, adapter,
  // endpoint, dof, ioCount} for the UI.
  Q_INVOKABLE QVariantList robotList() const;
  Q_INVOKABLE void saveRobot(const QString& name, const QString& adapter, const QString& endpoint);
  Q_INVOKABLE void loadRobot(const QString& id);
  Q_INVOKABLE void deleteRobot(const QString& id);
  // Current robot's IO channels as {name, kind, direction, variable, value, writable}
  // — the profile's declared banks (Y/M/AIN/AOT/GIN/GOT for PNR) with their live
  // telemetry value, for display and to drive the write controls.
  Q_INVOKABLE QVariantList ioChannels() const;
  // Write an IO channel (digital 0/1, analog scaled) through the adapter, so the
  // scene -> robot IO path stays in sync on a remote controller too.
  Q_INVOKABLE void writeIo(const QString& name, double value);

 signals:
  void telemetryChanged();
  void eventLogged(const QString& text);
  void phaseChanged(const QString& phase);
  void robotsChanged();

 private:
  void tick();
  void publish();
  // (Re)connect a robot: build its adapter (mock served the given profile, or TCP),
  // discover the profile, plan/validate/execute the demo. Shared by construction and
  // loadRobot so the scene mirrors any robot through the one ControllerAdapter seam.
  void connectRobot(const std::string& adapter, const std::string& endpoint,
                    const cavr::machine::MachineProfile& profile);
  void seed_default_robots();  // populate the registry with GP25 + PNR presets if empty

  // The robot is either the in-process mock or, when CAVR_ROBOT_ENDPOINT is set,
  // a remote controller reached over TCP (a cavr-robotd or a vendor bridge). The
  // ControllerAdapter interface is the only seam — the rest of this class is the
  // same either way, so the scene mirrors a real robot with no other changes.
  std::unique_ptr<cavr::adapter_sdk::ControllerAdapter> controller_;
  std::unique_ptr<cavr::catalog::SqliteProfileStore> registry_;  // named robots in SQLite
  bool remote_{false};
  bool manual_{false};  // set by jogging; suppresses the demo auto-restart
  cavr::machine::CoordinateSystem coord_sys_{cavr::machine::CoordinateSystem::Base};
  double speed_mm_s_{50.0};  // Cartesian jog speed
  cavr::runtime::SessionManager manager_;
  QTimer timer_;
  std::int64_t now_ns_{1'000'000'000};
  int run_index_{0};

  QVariantList joint_degrees_;
  QVariantList tcp_position_;
  QString phase_{"disconnected"};
  QString program_state_{"idle"};
  QString step_label_{"idle"};
  double speed_fraction_{0.0};
  bool weld_active_{false};
};
