#pragma once

#include "ProgramDocument.hpp"

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
#include <cavr/adapters/mock_camera/mock_camera.hpp>
#include <cavr/adapters/mock_robot/mock_controller.hpp>
#include <cavr/catalog/sqlite_profile_store.hpp>
#include <cavr/catalog/sqlite_program_store.hpp>
#include <cavr/core/geometry.hpp>
#include <cavr/machine/frames.hpp>
#include <cavr/machine/motion.hpp>
#include <cavr/runtime/session_manager.hpp>
#include <cavr/runtime/vision_guidance.hpp>

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
  // TCP readout in the selected coordinate system: [X, Y, Z (mm), Rx, Ry, Rz (deg)],
  // plus the coordinate-system name — for the viewport overlay and the pendant.
  Q_PROPERTY(QVariantList tcpCoords READ tcpCoords NOTIFY telemetryChanged)
  Q_PROPERTY(QString coordSystem READ coordSystem NOTIFY telemetryChanged)
  Q_PROPERTY(int selectedProgramStep READ selectedProgramStep NOTIFY programSelectionChanged)

 public:
  explicit RobotController(QObject* parent = nullptr);

  [[nodiscard]] QVariantList jointDegrees() const { return joint_degrees_; }
  [[nodiscard]] QString phase() const { return phase_; }
  [[nodiscard]] QString programState() const { return program_state_; }
  [[nodiscard]] QString stepLabel() const { return step_label_; }
  [[nodiscard]] double speedFraction() const { return speed_fraction_; }
  [[nodiscard]] bool weldActive() const { return weld_active_; }
  [[nodiscard]] QVariantList tcpPosition() const { return tcp_position_; }
  [[nodiscard]] QVariantList tcpCoords() const { return tcp_coords_; }
  [[nodiscard]] QString coordSystem() const { return coord_system_name_; }
  [[nodiscard]] int selectedProgramStep() const { return program_.selected_step(); }

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
  // Run a short circular arc (MoveC) from the current TCP through an offset via to
  // an offset end, so the arc motion is visible in the scene.
  Q_INVOKABLE void jogArc();
  Q_INVOKABLE void setCoordinateSystem(int system);        // 0 World, 1 Base, 2 Tool, 3 User
  Q_INVOKABLE void setSpeedMmS(double mm_s);               // Cartesian jog speed
  Q_INVOKABLE void selectTool(int slot);                   // choose the active tool
  Q_INVOKABLE void calibrateTool(int slot, double x_m, double y_m, double z_m);  // set TCP offset
  Q_INVOKABLE void clearTool(int slot);
  Q_INVOKABLE void runDemo();                              // leave manual mode, resume the demo
  Q_INVOKABLE bool saveSession(const QString& path);

  // Program editor: build an ordered job of steps by teaching from the live robot,
  // reorder/remove steps, run it on the twin, and save/load it by name in the DB —
  // job authoring, mirroring the robot registry.
  Q_INVOKABLE void addMoveL();                 // current TCP pose as a MoveL step
  Q_INVOKABLE void addMoveJ();                 // current joints as a MoveJ step
  Q_INVOKABLE void setVia();                   // stash the current pose as the next MoveC via
  Q_INVOKABLE void addMoveC();                 // arc through the stashed via to the current pose
  Q_INVOKABLE void addWait(double seconds);
  Q_INVOKABLE void addIoSet(const QString& channel, double value);
  Q_INVOKABLE void addToolOn();
  Q_INVOKABLE void addToolOff();
  Q_INVOKABLE void removeStep(int index);
  Q_INVOKABLE void moveStep(int index, int delta);  // reorder: -1 up, +1 down
  Q_INVOKABLE void clearProgram();
  Q_INVOKABLE void runProgram();
  Q_INVOKABLE QVariantList programSteps() const;    // [{index, kind, detail, status, ...}]
  Q_INVOKABLE void selectProgramStep(int index);
  // Pre-flight validation of the current program against the active robot: axis
  // limits, reachability (IK), frames. Summary string + overall ok flag.
  Q_INVOKABLE QString validationSummary() const;
  Q_INVOKABLE bool programValid() const;
  // Collision status of the current program against the active robot: self-collision
  // and floor penetration over the sampled trajectory (see libs/validation).
  Q_INVOKABLE QString collisionSummary() const;

  // Vision guidance: the live scan (a synchronized PointCloud from the camera) is
  // placed in the base frame via the profile's hand-eye extrinsics and compared to
  // the planned seam. seamOffsetMm is the [dx,dy,dz] correction in millimetres;
  // applySeamCorrection shifts the program's Cartesian targets by it.
  Q_INVOKABLE bool hasScan() const;
  Q_INVOKABLE int scanPointCount() const;
  Q_INVOKABLE QVariantList seamOffsetMm() const;
  Q_INVOKABLE QString visionSummary() const;
  Q_INVOKABLE void applySeamCorrection();

  // Replay: load a recorded SessionLog (JSON) and scrub it through the scene. While
  // replaying, the viewport/overlay/joint table are driven from the file instead of
  // the live robot; the timeline slider seeks by fraction of the recording.
  Q_INVOKABLE bool loadReplay(const QString& path);
  Q_INVOKABLE void replaySeek(double fraction);   // 0..1 of the recording
  Q_INVOKABLE void replayPlayPause();
  Q_INVOKABLE void exitReplay();                   // return to the live robot
  Q_INVOKABLE bool isReplaying() const { return replaying_; }
  Q_INVOKABLE bool replayPlaying() const { return replay_playing_; }
  Q_INVOKABLE double replayPositionFraction() const;
  Q_INVOKABLE QString replayInfo() const;          // "file • frame N/M • t / dur"

  // Saved jobs in the DB.
  Q_INVOKABLE QVariantList savedPrograms() const;   // [{id, name, steps}]
  Q_INVOKABLE void saveProgram(const QString& name);
  Q_INVOKABLE void loadProgram(const QString& id);
  Q_INVOKABLE void deleteProgram(const QString& id);

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
  // Per-axis joint state vs its configured limits (degrees): {name, value, lower,
  // upper, over}. Drives the pendant's joint-limit table.
  Q_INVOKABLE QVariantList jointStatus() const;
  // Write an IO channel (digital 0/1, analog scaled) through the adapter, so the
  // scene -> robot IO path stays in sync on a remote controller too.
  Q_INVOKABLE void writeIo(const QString& name, double value);

 signals:
  void telemetryChanged();
  void eventLogged(const QString& text);
  void phaseChanged(const QString& phase);
  void robotsChanged();
  void programChanged();          // the editable step list changed
  void savedProgramsChanged();    // the DB list of saved jobs changed
  void programSelectionChanged();  // selected step in list/timeline changed
  void replayChanged();            // replay loaded / seeked / play state changed

 private:
  void tick();
  void publish();
  // The frame/profile currently driving the scene: the loaded recording while
  // replaying, otherwise the live robot.
  [[nodiscard]] const cavr::adapter_sdk::RobotState& active_frame() const;
  [[nodiscard]] const cavr::machine::MachineProfile& active_profile() const;
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
  std::unique_ptr<cavr::catalog::SqliteProgramStore> programs_;  // named jobs in SQLite
  bool remote_{false};
  bool manual_{false};  // set by jogging; suppresses the demo auto-restart
  cavr::machine::CoordinateSystem coord_sys_{cavr::machine::CoordinateSystem::Base};
  double speed_mm_s_{50.0};  // Cartesian jog speed
  cavr::runtime::SessionManager manager_;
  // Synchronized vision source: streams a scan PointCloud on the same tick as the
  // robot during a running session, feeding the vision-guided seam correction.
  cavr::adapters::mock_camera::MockCamera camera_{8, 8, "weld_cam"};

  // Replay state: a loaded recording scrubbed through the scene by wall-clock offset.
  bool replaying_{false};
  bool replay_playing_{false};
  double replay_pos_s_{0.0};
  QString replay_file_;
  cavr::runtime::SessionLog replay_log_;
  // Private helper: the seam correction in metres for the current scan + program.
  [[nodiscard]] cavr::core::Vec3 seam_offset_m() const;
  ProgramDocument program_;
  bool running_current_program_{false};
  QTimer timer_;
  std::int64_t now_ns_{1'000'000'000};
  int run_index_{0};

  QVariantList joint_degrees_;
  QVariantList tcp_position_;
  QVariantList tcp_coords_;                 // [X,Y,Z mm, Rx,Ry,Rz deg] in coord_sys_
  QString coord_system_name_{"Base"};
  QString phase_{"disconnected"};
  QString program_state_{"idle"};
  QString step_label_{"idle"};
  double speed_fraction_{0.0};
  bool weld_active_{false};
};
