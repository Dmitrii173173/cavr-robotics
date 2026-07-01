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
#include <cavr/runtime/session_manager.hpp>

#include <cstdint>
#include <memory>

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
  // to move to its home joint pose right now, interrupting the running program.
  Q_INVOKABLE void jogHome();
  Q_INVOKABLE bool saveSession(const QString& path);

 signals:
  void telemetryChanged();
  void eventLogged(const QString& text);
  void phaseChanged(const QString& phase);

 private:
  void tick();
  void publish();

  // The robot is either the in-process mock or, when CAVR_ROBOT_ENDPOINT is set,
  // a remote controller reached over TCP (a cavr-robotd or a vendor bridge). The
  // ControllerAdapter interface is the only seam — the rest of this class is the
  // same either way, so the scene mirrors a real robot with no other changes.
  std::unique_ptr<cavr::adapter_sdk::ControllerAdapter> controller_;
  bool remote_{false};
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
