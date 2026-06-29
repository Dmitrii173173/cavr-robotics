#pragma once

#include <QMainWindow>

class QDockWidget;
class QWidget;
class QLabel;
class QListWidget;
class RobotController;

class StudioWindow final : public QMainWindow {
 public:
  explicit StudioWindow(QWidget* parent = nullptr);

 private:
  void configure_chrome();
  void create_docks();
  void apply_theme();

  [[nodiscard]] QWidget* create_robot_viewport();

  RobotController* controller_{nullptr};
  QListWidget* events_list_{nullptr};
  QLabel* status_phase_{nullptr};

  [[nodiscard]] QDockWidget* make_dock(const QString& title, QWidget* widget);
  [[nodiscard]] QWidget* make_session_panel();
  [[nodiscard]] QWidget* make_channels_panel();
  [[nodiscard]] QWidget* make_events_panel();
  [[nodiscard]] QWidget* make_telemetry_panel();
  [[nodiscard]] QWidget* make_calibration_panel();
  [[nodiscard]] QWidget* make_fault_panel();
};
