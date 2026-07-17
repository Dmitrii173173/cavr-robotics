#pragma once

#include <QMainWindow>

class QDockWidget;
class QWidget;
class QLabel;
class QListWidget;
class QLineEdit;
class QComboBox;
class QTableWidget;
class QSlider;
class QPushButton;
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

  // Robot registry panel widgets, refreshed from RobotController::robotsChanged.
  QListWidget* robot_list_{nullptr};
  QLineEdit* robot_name_{nullptr};
  QComboBox* robot_adapter_{nullptr};
  QLineEdit* robot_endpoint_{nullptr};
  QTableWidget* io_table_{nullptr};
  QComboBox* io_write_channel_{nullptr};  // writable channels for the write controls
  QLabel* pendant_lcd_{nullptr};          // teach-pendant LCD read-out
  QTableWidget* joint_table_{nullptr};    // per-axis position vs joint limits
  QListWidget* program_list_{nullptr};    // editable program steps
  QListWidget* saved_programs_list_{nullptr};  // saved jobs in the DB
  QLineEdit* program_name_{nullptr};      // name field for saving a job
  QLabel* validation_label_{nullptr};     // pre-flight validation summary
  QLabel* collision_label_{nullptr};      // collision status (self + floor)
  QLabel* cycle_label_{nullptr};          // estimated cycle time under the chosen profile
  QComboBox* motion_profile_{nullptr};    // trapezoidal / S-curve velocity profile
  QLabel* fault_status_label_{nullptr};   // live fault / alarm read-out
  QLabel* vision_label_{nullptr};         // live vision-guided seam correction read-out
  QLabel* calib_intrinsics_label_{nullptr};  // live camera intrinsics
  QLabel* calib_handeye_label_{nullptr};     // solved hand-eye transform + residual
  QLabel* calib_status_label_{nullptr};      // calibration sample/solve status
  QSlider* replay_slider_{nullptr};       // scrub position through a loaded recording
  QLabel* replay_label_{nullptr};         // replay read-out (file / frame / time)
  QPushButton* replay_play_{nullptr};     // play / pause the replay
  void refresh_robots();
  void refresh_io();       // live IO values + writable-channel list, on each telemetry tick
  void refresh_program();  // editable step list
  void refresh_saved_programs();  // saved-jobs list from the DB
  void refresh_joints();   // per-axis position vs joint limits, on each telemetry tick
  void update_pendant();   // refresh the pendant LCD read-out on each telemetry tick
  void refresh_calibration();  // live vision-guidance read-out, on each telemetry tick
  void refresh_replay();       // sync the replay slider/label with the controller state
  void refresh_faults();       // live fault/alarm read-out, on each telemetry tick

  [[nodiscard]] QDockWidget* make_dock(const QString& title, QWidget* widget);
  [[nodiscard]] QWidget* make_robots_panel();
  [[nodiscard]] QWidget* make_program_panel();
  [[nodiscard]] QWidget* make_session_panel();
  [[nodiscard]] QWidget* make_channels_panel();
  [[nodiscard]] QWidget* make_events_panel();
  [[nodiscard]] QWidget* make_telemetry_panel();
  [[nodiscard]] QWidget* make_calibration_panel();
  [[nodiscard]] QWidget* make_fault_panel();
  [[nodiscard]] QWidget* make_jog_panel();
};
