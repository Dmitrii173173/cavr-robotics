#include "StudioWindow.hpp"

#include "CameraView.hpp"
#include "RobotController.hpp"
#include "TimelineWidget.hpp"

#include <functional>

#include <QAction>
#include <QQmlContext>
#include <QQuickWidget>
#include <QUrl>
#include <QComboBox>
#include <QDockWidget>
#include <QScrollArea>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QSplitter>
#include <QStatusBar>
#include <QTabWidget>
#include <QTableWidget>
#include <QColor>
#include <QToolBar>
#include <QVariant>
#include <QVariantList>
#include <QVariantMap>
#include <QVBoxLayout>

namespace {

QLabel* value_label(const QString& text) {
  auto* label = new QLabel(text);
  label->setTextInteractionFlags(Qt::TextSelectableByMouse);
  return label;
}

QFrame* horizontal_rule() {
  auto* line = new QFrame;
  line->setFrameShape(QFrame::HLine);
  line->setFrameShadow(QFrame::Sunken);
  return line;
}

}  // namespace

StudioWindow::StudioWindow(QWidget* parent) : QMainWindow(parent) {
  setWindowTitle("CAVR Studio");
  resize(1536, 1024);
  setMinimumSize(1180, 760);

  controller_ = new RobotController(this);

  configure_chrome();
  setCentralWidget(create_robot_viewport());
  create_docks();
  apply_theme();

  // live telemetry -> Events dock + status bar
  connect(controller_, &RobotController::eventLogged, this, [this](const QString& text) {
    if (!events_list_) return;
    events_list_->addItem(text);
    events_list_->scrollToBottom();
    while (events_list_->count() > 200) delete events_list_->takeItem(0);
  });
  connect(controller_, &RobotController::phaseChanged, this, [this](const QString& phase) {
    if (status_phase_) status_phase_->setText("Phase: " + phase);
  });
  // Registry changes (save/load/delete) repopulate the robot list + IO table.
  connect(controller_, &RobotController::robotsChanged, this, [this] { refresh_robots(); });
  // Editing the program repopulates the step list; DB changes the saved-jobs list.
  connect(controller_, &RobotController::programChanged, this, [this] { refresh_program(); });
  connect(controller_, &RobotController::savedProgramsChanged, this,
          [this] { refresh_saved_programs(); });
  // Each telemetry tick refreshes the live IO values and the pendant read-out.
  connect(controller_, &RobotController::telemetryChanged, this, [this] {
    refresh_io();
    update_pendant();
  });
  refresh_robots();
  refresh_program();
  refresh_saved_programs();
  update_pendant();
}

QWidget* StudioWindow::create_robot_viewport() {
  auto* viewport = new QQuickWidget(this);
  viewport->setResizeMode(QQuickWidget::SizeRootObjectToView);
  viewport->setMinimumSize(520, 380);

  const QString glb =
      QString::fromUtf8(CAVR_ASSETS_DIR) + "/robots/yaskawa_gp25/gp25.glb";
  viewport->rootContext()->setContextProperty("robotUrl", QUrl::fromLocalFile(glb));
  viewport->rootContext()->setContextProperty("robot", controller_);
  viewport->setSource(QUrl("qrc:/qml/RobotViewport.qml"));
  return viewport;
}

void StudioWindow::configure_chrome() {
  setDockOptions(QMainWindow::AnimatedDocks | QMainWindow::AllowNestedDocks | QMainWindow::AllowTabbedDocks);

  auto* toolbar = addToolBar("Workspace");
  toolbar->setMovable(false);
  toolbar->addWidget(new QLabel("  CAVR Studio  "));
  toolbar->addSeparator();
  toolbar->addAction("Record");
  auto* replay = toolbar->addAction("Replay");
  replay->setCheckable(true);
  replay->setChecked(true);
  toolbar->addAction("Validate");
  toolbar->addAction("Inspect");
  toolbar->addSeparator();
  // Live jog (scene -> robot): commands the robot — in-process mock or, with
  // CAVR_ROBOT_ENDPOINT set, a remote one — to move home right now.
  auto* jog = toolbar->addAction("Jog Home");
  connect(jog, &QAction::triggered, this, [this] { controller_->jogHome(); });
  toolbar->addSeparator();
  toolbar->addWidget(new QLabel("weld_scan_2025_05_10.mcap"));

  status_phase_ = new QLabel("Phase: starting");
  statusBar()->addWidget(status_phase_);
  statusBar()->addPermanentWidget(new QLabel("CPU 18%"));
  statusBar()->addPermanentWidget(new QLabel("RAM 2.1 GB"));
  statusBar()->addPermanentWidget(new QLabel("Dropped camera: 12 (0.02%)"));
}

void StudioWindow::create_docks() {
  // Visual-Studio-style tool windows: each dock area holds a group of tabbed
  // panels (one visible at a time, all reachable by tab) instead of many panels
  // squeezed into vertical stacks. Tabs sit at the top of each group.
  setTabPosition(Qt::AllDockWidgetAreas, QTabWidget::North);

  auto* robots = make_dock("Robots", make_robots_panel());
  auto* jog = make_dock("Jog + Tools", make_jog_panel());
  auto* program = make_dock("Program", make_program_panel());
  auto* session = make_dock("Session", make_session_panel());
  auto* channels = make_dock("Channels", make_channels_panel());
  auto* calibration = make_dock("Calibration", make_calibration_panel());

  auto* camera = make_dock("Camera", new CameraView(this));
  auto* telemetry = make_dock("Telemetry", make_telemetry_panel());
  auto* faults = make_dock("Faults", make_fault_panel());

  auto* events = make_dock("Events", make_events_panel());
  auto* timeline = make_dock("Timeline", new TimelineWidget(this));

  // Left group: robot registry, jog/tools, session, channels, calibration.
  addDockWidget(Qt::LeftDockWidgetArea, robots);
  for (auto* d : {jog, program, session, channels, calibration}) {
    addDockWidget(Qt::LeftDockWidgetArea, d);
    tabifyDockWidget(robots, d);
  }

  // Right group: camera view, telemetry, fault injection.
  addDockWidget(Qt::RightDockWidgetArea, camera);
  for (auto* d : {telemetry, faults}) {
    addDockWidget(Qt::RightDockWidgetArea, d);
    tabifyDockWidget(camera, d);
  }

  // Bottom group: events log and the timeline.
  addDockWidget(Qt::BottomDockWidgetArea, events);
  addDockWidget(Qt::BottomDockWidgetArea, timeline);
  tabifyDockWidget(events, timeline);
  timeline->setMinimumHeight(200);

  // Open each group on its primary tab.
  robots->raise();
  camera->raise();
  events->raise();

  // Give the 3D viewport the lion's share of width, like the VS editor area.
  resizeDocks({robots, camera}, {340, 380}, Qt::Horizontal);
}

QDockWidget* StudioWindow::make_dock(const QString& title, QWidget* widget) {
  auto* dock = new QDockWidget(title, this);
  dock->setObjectName(title);
  dock->setWidget(widget);
  dock->setAllowedAreas(Qt::AllDockWidgetAreas);
  return dock;
}

QWidget* StudioWindow::make_robots_panel() {
  auto* panel = new QWidget;
  auto* layout = new QVBoxLayout(panel);

  // Saved robots. Selecting one fills the editor + IO table; Load connects it.
  layout->addWidget(new QLabel("Registered robots"));
  robot_list_ = new QListWidget;
  layout->addWidget(robot_list_);

  auto* row_buttons = new QHBoxLayout;
  auto* load = new QPushButton("Load");
  auto* del = new QPushButton("Delete");
  row_buttons->addWidget(load);
  row_buttons->addWidget(del);
  layout->addLayout(row_buttons);

  connect(robot_list_, &QListWidget::currentItemChanged, this, [this](QListWidgetItem* item) {
    if (!item || !robot_name_) return;
    robot_name_->setText(item->data(Qt::UserRole + 1).toString());       // name
    robot_endpoint_->setText(item->data(Qt::UserRole + 3).toString());   // endpoint
    const QString adapter = item->data(Qt::UserRole + 2).toString();
    const int idx = robot_adapter_->findText(adapter);
    if (idx >= 0) robot_adapter_->setCurrentIndex(idx);
  });
  connect(load, &QPushButton::clicked, this, [this] {
    if (auto* item = robot_list_->currentItem())
      controller_->loadRobot(item->data(Qt::UserRole).toString());
  });
  connect(del, &QPushButton::clicked, this, [this] {
    if (auto* item = robot_list_->currentItem())
      controller_->deleteRobot(item->data(Qt::UserRole).toString());
  });

  layout->addWidget(horizontal_rule());

  // Editor: name the current (or a new) robot and save it to the registry.
  layout->addWidget(new QLabel("Add / save robot"));
  auto* form = new QFormLayout;
  robot_name_ = new QLineEdit;
  robot_name_->setPlaceholderText("Robot name");
  robot_adapter_ = new QComboBox;
  robot_adapter_->addItems({"mock", "generic_tcp"});
  robot_endpoint_ = new QLineEdit;
  robot_endpoint_->setPlaceholderText("host:port (for generic_tcp)");
  form->addRow("Name", robot_name_);
  form->addRow("Adapter", robot_adapter_);
  form->addRow("Endpoint", robot_endpoint_);
  layout->addLayout(form);

  auto* save = new QPushButton("Save current as…");
  connect(save, &QPushButton::clicked, this, [this] {
    controller_->saveRobot(robot_name_->text(), robot_adapter_->currentText(),
                           robot_endpoint_->text());
  });
  layout->addWidget(save);

  layout->addWidget(horizontal_rule());

  // IO banks of the currently connected robot (Y/M/AIN/AOT/GIN/GOT for PNR), with
  // their live value, updated on every telemetry tick.
  layout->addWidget(new QLabel("IO channels (live)"));
  io_table_ = new QTableWidget(0, 4);
  io_table_->setHorizontalHeaderLabels({"Name", "Dir", "Variable", "Value"});
  io_table_->verticalHeader()->hide();
  io_table_->horizontalHeader()->setStretchLastSection(true);
  io_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  layout->addWidget(io_table_);

  // Write controls (scene -> robot): pick a writable channel, set a value.
  auto* write_row = new QHBoxLayout;
  io_write_channel_ = new QComboBox;
  auto* io_value = new QDoubleSpinBox;
  io_value->setRange(-1000.0, 1000.0);
  io_value->setDecimals(2);
  auto* set0 = new QPushButton("0");
  auto* set1 = new QPushButton("1");
  auto* write = new QPushButton("Write");
  write_row->addWidget(io_write_channel_, 1);
  write_row->addWidget(io_value);
  write_row->addWidget(set0);
  write_row->addWidget(set1);
  write_row->addWidget(write);
  layout->addLayout(write_row);

  connect(set0, &QPushButton::clicked, this, [this] {
    if (io_write_channel_->count() > 0) controller_->writeIo(io_write_channel_->currentText(), 0.0);
  });
  connect(set1, &QPushButton::clicked, this, [this] {
    if (io_write_channel_->count() > 0) controller_->writeIo(io_write_channel_->currentText(), 1.0);
  });
  connect(write, &QPushButton::clicked, this, [this, io_value] {
    if (io_write_channel_->count() > 0)
      controller_->writeIo(io_write_channel_->currentText(), io_value->value());
  });

  auto* scroll = new QScrollArea;
  scroll->setWidget(panel);
  scroll->setWidgetResizable(true);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  return scroll;
}

void StudioWindow::refresh_robots() {
  if (!robot_list_ || !controller_) return;

  const QString selected =
      robot_list_->currentItem() ? robot_list_->currentItem()->data(Qt::UserRole).toString()
                                 : QString();
  robot_list_->clear();
  const QVariantList robots = controller_->robotList();
  for (const QVariant& entry : robots) {
    const QVariantMap m = entry.toMap();
    const QString label = QString("%1  —  %2  (%3 axes, %4 IO)")
                              .arg(m.value("name").toString(), m.value("model").toString())
                              .arg(m.value("dof").toInt())
                              .arg(m.value("ioCount").toInt());
    auto* item = new QListWidgetItem(label, robot_list_);
    item->setData(Qt::UserRole, m.value("id"));
    item->setData(Qt::UserRole + 1, m.value("name"));
    item->setData(Qt::UserRole + 2, m.value("adapter"));
    item->setData(Qt::UserRole + 3, m.value("endpoint"));
    if (m.value("id").toString() == selected) robot_list_->setCurrentItem(item);
  }

  // The connected robot may have changed: rebuild the writable-channel list.
  if (io_write_channel_) io_write_channel_->clear();
  refresh_io();
}

void StudioWindow::refresh_io() {
  if (!io_table_ || !controller_) return;
  const QVariantList io = controller_->ioChannels();

  // Repopulate the writable-channel combo only when the set of channels changes
  // (a robot switch), so it doesn't reset the user's selection every tick.
  if (io_write_channel_ && io_write_channel_->count() == 0) {
    for (const QVariant& entry : io) {
      const QVariantMap c = entry.toMap();
      if (c.value("writable").toBool()) io_write_channel_->addItem(c.value("name").toString());
    }
  }

  io_table_->setRowCount(static_cast<int>(io.size()));
  for (int r = 0; r < io.size(); ++r) {
    const QVariantMap c = io[r].toMap();
    io_table_->setItem(r, 0, new QTableWidgetItem(c.value("name").toString()));
    io_table_->setItem(r, 1, new QTableWidgetItem(c.value("direction").toString()));
    io_table_->setItem(r, 2, new QTableWidgetItem(c.value("variable").toString()));

    const double value = c.value("value").toDouble();
    const bool digital = c.value("kind").toString() == "digital" || c.value("kind").toString() == "group";
    auto* cell = new QTableWidgetItem(digital ? (value > 0.5 ? "ON" : "off")
                                              : QString::number(value, 'g', 4));
    if (digital && value > 0.5) cell->setForeground(QColor("#4dd06a"));  // lit output = green
    io_table_->setItem(r, 3, cell);
  }
}

QWidget* StudioWindow::make_program_panel() {
  auto* panel = new QWidget;
  auto* layout = new QVBoxLayout(panel);

  // The editable job: an ordered list of steps taught from the live robot.
  layout->addWidget(new QLabel("Program steps"));
  program_list_ = new QListWidget;
  layout->addWidget(program_list_);

  // Add-step buttons: teach motion from the current pose/joints, or insert
  // Wait/Tool steps. MoveC needs a via first (Set via), then Add MoveC.
  auto* add_grid = new QGridLayout;
  const struct {
    const char* label;
    std::function<void()> action;
  } adders[] = {
      {"+ MoveJ", [this] { controller_->addMoveJ(); }},
      {"+ MoveL", [this] { controller_->addMoveL(); }},
      {"Set via", [this] { controller_->setVia(); }},
      {"+ MoveC", [this] { controller_->addMoveC(); }},
      {"+ ToolOn", [this] { controller_->addToolOn(); }},
      {"+ ToolOff", [this] { controller_->addToolOff(); }},
  };
  int gi = 0;
  for (const auto& a : adders) {
    auto* b = new QPushButton(a.label);
    const auto act = a.action;
    connect(b, &QPushButton::clicked, this, [act] { act(); });
    add_grid->addWidget(b, gi / 2, gi % 2);
    ++gi;
  }
  layout->addLayout(add_grid);

  // Wait step with a seconds spin.
  auto* wait_row = new QHBoxLayout;
  auto* wait_spin = new QDoubleSpinBox;
  wait_spin->setRange(0.0, 60.0);
  wait_spin->setValue(1.0);
  wait_spin->setSuffix(" s");
  auto* add_wait = new QPushButton("+ Wait");
  connect(add_wait, &QPushButton::clicked, this,
          [this, wait_spin] { controller_->addWait(wait_spin->value()); });
  wait_row->addWidget(wait_spin);
  wait_row->addWidget(add_wait);
  layout->addLayout(wait_row);

  // Reorder / remove / clear / run.
  auto* ops = new QHBoxLayout;
  auto* up = new QPushButton("↑");
  auto* down = new QPushButton("↓");
  auto* remove = new QPushButton("Remove");
  auto* clear = new QPushButton("Clear");
  connect(up, &QPushButton::clicked, this, [this] {
    if (auto* it = program_list_->currentItem()) controller_->moveStep(program_list_->row(it), -1);
  });
  connect(down, &QPushButton::clicked, this, [this] {
    if (auto* it = program_list_->currentItem()) controller_->moveStep(program_list_->row(it), +1);
  });
  connect(remove, &QPushButton::clicked, this, [this] {
    if (auto* it = program_list_->currentItem()) controller_->removeStep(program_list_->row(it));
  });
  connect(clear, &QPushButton::clicked, this, [this] { controller_->clearProgram(); });
  ops->addWidget(up);
  ops->addWidget(down);
  ops->addWidget(remove);
  ops->addWidget(clear);
  layout->addLayout(ops);

  auto* run = new QPushButton("▶ Run program");
  connect(run, &QPushButton::clicked, this, [this] { controller_->runProgram(); });
  layout->addWidget(run);

  layout->addWidget(horizontal_rule());

  // Save / load named jobs from the DB (mirrors the robot registry).
  layout->addWidget(new QLabel("Saved jobs"));
  saved_programs_list_ = new QListWidget;
  saved_programs_list_->setMaximumHeight(120);
  layout->addWidget(saved_programs_list_);

  auto* save_row = new QHBoxLayout;
  program_name_ = new QLineEdit;
  program_name_->setPlaceholderText("Job name");
  auto* save = new QPushButton("Save");
  connect(save, &QPushButton::clicked, this,
          [this] { controller_->saveProgram(program_name_->text()); });
  save_row->addWidget(program_name_, 1);
  save_row->addWidget(save);
  layout->addLayout(save_row);

  auto* db_ops = new QHBoxLayout;
  auto* load = new QPushButton("Load");
  auto* del = new QPushButton("Delete");
  connect(load, &QPushButton::clicked, this, [this] {
    if (auto* it = saved_programs_list_->currentItem())
      controller_->loadProgram(it->data(Qt::UserRole).toString());
  });
  connect(del, &QPushButton::clicked, this, [this] {
    if (auto* it = saved_programs_list_->currentItem())
      controller_->deleteProgram(it->data(Qt::UserRole).toString());
  });
  db_ops->addWidget(load);
  db_ops->addWidget(del);
  layout->addLayout(db_ops);

  auto* scroll = new QScrollArea;
  scroll->setWidget(panel);
  scroll->setWidgetResizable(true);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  return scroll;
}

void StudioWindow::refresh_program() {
  if (!program_list_ || !controller_) return;
  program_list_->clear();
  for (const QVariant& entry : controller_->programSteps()) {
    const QVariantMap m = entry.toMap();
    program_list_->addItem(QString("%1.  %2   %3")
                               .arg(m.value("index").toInt(), 2)
                               .arg(m.value("kind").toString().toUpper(), -8)
                               .arg(m.value("detail").toString()));
  }
}

void StudioWindow::refresh_saved_programs() {
  if (!saved_programs_list_ || !controller_) return;
  saved_programs_list_->clear();
  for (const QVariant& entry : controller_->savedPrograms()) {
    const QVariantMap m = entry.toMap();
    auto* item = new QListWidgetItem(
        QString("%1  (%2 steps)").arg(m.value("name").toString()).arg(m.value("steps").toInt()),
        saved_programs_list_);
    item->setData(Qt::UserRole, m.value("id"));
  }
}

void StudioWindow::update_pendant() {
  if (!pendant_lcd_ || !controller_) return;
  const QVariantList c = controller_->tcpCoords();
  const auto v = [&](int i) { return c.size() > i ? c[i].toDouble() : 0.0; };
  const auto col = [](double x) { return QString::number(x, 'f', 1).rightJustified(8); };

  const QString text =
      QString("  %1 FRAME        %2\n").arg(controller_->coordSystem().toUpper()).arg(controller_->programState().toUpper()) +
      QString("  X %1   Rx %2\n").arg(col(v(0))).arg(col(v(3))) +
      QString("  Y %1   Ry %2\n").arg(col(v(1))).arg(col(v(4))) +
      QString("  Z %1   Rz %2\n").arg(col(v(2))).arg(col(v(5))) +
      QString("  ------------------------------\n") +
      QString("  step: %1").arg(controller_->stepLabel());
  pendant_lcd_->setText(text);
}

QWidget* StudioWindow::make_session_panel() {
  auto* panel = new QWidget;
  auto* layout = new QFormLayout(panel);
  layout->addRow("File", value_label("weld_scan_2025_05_10.mcap"));
  layout->addRow("Duration", value_label("00:02:34.893"));
  layout->addRow("Start Time", value_label("2025-05-10 14:23:11.123"));
  layout->addRow("End Time", value_label("2025-05-10 14:25:46.016"));
  layout->addRow("Messages", value_label("1 234 567"));
  layout->addRow("Size", value_label("2.45 GB"));
  layout->addRow("Version", value_label("0.1.0"));
  return panel;
}

QWidget* StudioWindow::make_channels_panel() {
  auto* list = new QListWidget;
  const QStringList channels = {
      "/robot/state", "/robot/joints", "/robot/command", "/camera/rgb",
      "/camera/depth", "/camera/pointcloud", "/transforms", "/calibration",
      "/io/digital", "/io/analog", "/session/event"};
  for (const auto& channel : channels) {
    auto* item = new QListWidgetItem(channel, list);
    item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
    item->setCheckState(channel.contains("command") || channel.contains("io/") ? Qt::Unchecked : Qt::Checked);
  }
  return list;
}

QWidget* StudioWindow::make_events_panel() {
  auto* list = new QListWidget;
  list->addItem("session_started | live telemetry from mock controller");
  events_list_ = list;
  return list;
}

QWidget* StudioWindow::make_telemetry_panel() {
  auto* table = new QTableWidget(7, 2);
  table->setHorizontalHeaderLabels({"Field", "Value"});
  table->verticalHeader()->hide();
  table->horizontalHeader()->setStretchLastSection(true);
  table->setEditTriggers(QAbstractItemView::NoEditTriggers);

  const QList<QPair<QString, QString>> rows = {
      {"Position (m)", "X 0.523, Y 0.152, Z 0.812"},
      {"Orientation (rad)", "Rx 0.215, Ry -1.570, Rz 0.785"},
      {"Quaternion", "0.707, 0.000, 0.707, 0.000"},
      {"Speed", "0.125 m/s"},
      {"Mode", "RUNNING"},
      {"Program", "WELD_SCAN"},
      {"Line", "42"},
  };

  for (int row = 0; row < rows.size(); ++row) {
    table->setItem(row, 0, new QTableWidgetItem(rows[row].first));
    table->setItem(row, 1, new QTableWidgetItem(rows[row].second));
  }
  return table;
}

QWidget* StudioWindow::make_calibration_panel() {
  auto* panel = new QWidget;
  auto* layout = new QFormLayout(panel);
  layout->addRow("Camera Intrinsics", value_label("cam_01_intrinsics.yaml"));
  layout->addRow("Hand-Eye", value_label("he_2025_05_10.yaml"));
  layout->addRow("Reprojection Error", value_label("0.42 px"));
  layout->addRow("Status", value_label("Valid"));
  return panel;
}

QWidget* StudioWindow::make_jog_panel() {
  constexpr double kDeg5 = 5.0 * 3.14159265358979323846 / 180.0;  // 5° in radians

  auto* panel = new QWidget;
  panel->setObjectName("pendant");
  auto* layout = new QVBoxLayout(panel);
  layout->setContentsMargins(10, 10, 10, 10);

  // Pendant header + LCD read-out: the live TCP pose, coordinate system, active
  // tool and program state, like the screen at the top of a real teach pendant.
  auto* title = new QLabel("TEACH PENDANT");
  title->setObjectName("pendantTitle");
  title->setAlignment(Qt::AlignCenter);
  layout->addWidget(title);

  pendant_lcd_ = new QLabel;
  pendant_lcd_->setObjectName("pendantLcd");
  pendant_lcd_->setTextFormat(Qt::PlainText);
  pendant_lcd_->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  layout->addWidget(pendant_lcd_);

  layout->addWidget(horizontal_rule());

  // Coordinate system for Cartesian jog: World / Base / Tool / User.
  auto* frame_row = new QHBoxLayout;
  frame_row->addWidget(new QLabel("Frame"));
  auto* frame = new QComboBox;
  frame->addItems({"World", "Base", "Tool", "User"});
  frame->setCurrentIndex(1);  // Base
  connect(frame, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          [this](int i) { controller_->setCoordinateSystem(i); });
  frame_row->addWidget(frame);
  layout->addLayout(frame_row);

  // Cartesian jog speed, in mm/s.
  auto* speed_row = new QHBoxLayout;
  speed_row->addWidget(new QLabel("Speed mm/s"));
  auto* speed = new QDoubleSpinBox;
  speed->setRange(1.0, 2000.0);
  speed->setValue(50.0);
  connect(speed, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          [this](double v) { controller_->setSpeedMmS(v); });
  speed_row->addWidget(speed);
  layout->addLayout(speed_row);

  layout->addWidget(horizontal_rule());

  // Per-axis joint jog: each row is  name  [ - ]  [ + ].
  layout->addWidget(new QLabel("Joint jog (±5°)"));
  const QStringList axes = {"S", "L", "U", "R", "B", "T"};
  for (int i = 0; i < axes.size(); ++i) {
    auto* row = new QHBoxLayout;
    row->addWidget(new QLabel(axes[i]));
    auto* minus = new QPushButton("−");
    auto* plus = new QPushButton("+");
    connect(minus, &QPushButton::clicked, this, [this, i] { controller_->jogJoint(i, -5.0); });
    connect(plus, &QPushButton::clicked, this, [this, i] { controller_->jogJoint(i, 5.0); });
    row->addWidget(minus);
    row->addWidget(plus);
    layout->addLayout(row);
  }

  layout->addWidget(horizontal_rule());

  // Cartesian jog in the selected frame: X/Y/Z (±5 cm) and Rx/Ry/Rz (±5°), IK-solved.
  layout->addWidget(new QLabel("Cartesian jog (±5 cm / ±5°)"));
  const struct {
    const char* label;
    double tx, ty, tz, rx, ry, rz;
  } cart[] = {
      {"X", 0.05, 0, 0, 0, 0, 0}, {"Y", 0, 0.05, 0, 0, 0, 0},   {"Z", 0, 0, 0.05, 0, 0, 0},
      {"Rx", 0, 0, 0, kDeg5, 0, 0}, {"Ry", 0, 0, 0, 0, kDeg5, 0}, {"Rz", 0, 0, 0, 0, 0, kDeg5}};
  for (const auto& c : cart) {
    auto* row = new QHBoxLayout;
    row->addWidget(new QLabel(c.label));
    auto* minus = new QPushButton("−");
    auto* plus = new QPushButton("+");
    const double tx = c.tx, ty = c.ty, tz = c.tz, rx = c.rx, ry = c.ry, rz = c.rz;
    connect(minus, &QPushButton::clicked, this,
            [this, tx, ty, tz, rx, ry, rz] { controller_->jogCartesian(-tx, -ty, -tz, -rx, -ry, -rz); });
    connect(plus, &QPushButton::clicked, this,
            [this, tx, ty, tz, rx, ry, rz] { controller_->jogCartesian(tx, ty, tz, rx, ry, rz); });
    row->addWidget(minus);
    row->addWidget(plus);
    layout->addLayout(row);
  }

  layout->addWidget(horizontal_rule());

  // Tool table: select a slot (0–9) and calibrate its TCP offset or clear it.
  layout->addWidget(new QLabel("Tools (10 slots)"));
  auto* tool_row = new QHBoxLayout;
  tool_row->addWidget(new QLabel("Slot"));
  auto* tool = new QComboBox;
  for (int i = 0; i < 10; ++i) tool->addItem(QString::number(i));
  connect(tool, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          [this](int i) { controller_->selectTool(i); });
  tool_row->addWidget(tool);
  layout->addLayout(tool_row);

  auto* cal_row = new QHBoxLayout;
  cal_row->addWidget(new QLabel("TCP"));
  auto make_tcp_spin = [](double init) {
    auto* s = new QDoubleSpinBox;
    s->setRange(-1.0, 1.0);
    s->setSingleStep(0.01);
    s->setDecimals(3);
    s->setValue(init);
    return s;
  };
  auto* tcp_x = make_tcp_spin(0.0);
  auto* tcp_y = make_tcp_spin(0.0);
  auto* tcp_z = make_tcp_spin(0.101);
  cal_row->addWidget(tcp_x);
  cal_row->addWidget(tcp_y);
  cal_row->addWidget(tcp_z);
  layout->addLayout(cal_row);

  auto* tool_btns = new QHBoxLayout;
  auto* calibrate = new QPushButton("Calibrate");
  auto* clear_tool = new QPushButton("Clear");
  connect(calibrate, &QPushButton::clicked, this, [this, tool, tcp_x, tcp_y, tcp_z] {
    controller_->calibrateTool(tool->currentIndex(), tcp_x->value(), tcp_y->value(), tcp_z->value());
  });
  connect(clear_tool, &QPushButton::clicked, this,
          [this, tool] { controller_->clearTool(tool->currentIndex()); });
  tool_btns->addWidget(calibrate);
  tool_btns->addWidget(clear_tool);
  layout->addLayout(tool_btns);

  layout->addWidget(horizontal_rule());
  auto* home = new QPushButton("Jog Home");
  auto* arc = new QPushButton("Arc (MoveC)");
  auto* demo = new QPushButton("Run Demo");
  connect(home, &QPushButton::clicked, this, [this] { controller_->jogHome(); });
  connect(arc, &QPushButton::clicked, this, [this] { controller_->jogArc(); });
  connect(demo, &QPushButton::clicked, this, [this] { controller_->runDemo(); });
  layout->addWidget(home);
  layout->addWidget(arc);
  layout->addWidget(demo);
  layout->addStretch();

  // The panel is tall; wrap it so it stays usable in a short dock.
  auto* scroll = new QScrollArea;
  scroll->setWidget(panel);
  scroll->setWidgetResizable(true);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  return scroll;
}

QWidget* StudioWindow::make_fault_panel() {
  auto* panel = new QWidget;
  auto* layout = new QVBoxLayout(panel);
  auto* form = new QFormLayout;
  form->addRow("Profile", value_label("latency_100ms_drop2"));
  form->addRow("Camera Delay", value_label("100 ms"));
  form->addRow("Drop Rate", value_label("2.0%"));
  form->addRow("Pose Noise (pos)", value_label("0.0 mm"));
  form->addRow("Pose Noise (rot)", value_label("0.0 deg"));
  layout->addLayout(form);
  layout->addWidget(horizontal_rule());
  layout->addWidget(new QPushButton("Configure"));
  layout->addStretch();
  return panel;
}

void StudioWindow::apply_theme() {
  setStyleSheet(R"qss(
    QMainWindow, QWidget {
      background: #10151b;
      color: #e6ecf2;
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      font-size: 13px;
    }
    QToolBar, QStatusBar {
      background: #111821;
      border: 1px solid #2b3947;
      spacing: 8px;
    }
    QDockWidget {
      titlebar-close-icon: none;
      titlebar-normal-icon: none;
      border: 1px solid #2b3947;
    }
    QDockWidget::title {
      background: #18212b;
      padding: 7px;
      text-transform: uppercase;
      border-bottom: 1px solid #2b3947;
    }
    QListWidget, QTableWidget, QLineEdit, QComboBox {
      background: #18212b;
      alternate-background-color: #1c2631;
      border: 1px solid #2b3947;
      border-radius: 4px;
      selection-background-color: #2f8cff;
    }
    QPushButton, QToolButton {
      background: #141c25;
      border: 1px solid #2b3947;
      border-radius: 4px;
      padding: 5px 10px;
    }
    QPushButton:hover, QToolButton:hover {
      border-color: #4d9dff;
    }
    QHeaderView::section {
      background: #1c2631;
      border: 1px solid #2b3947;
      padding: 4px;
    }
    QLabel {
      color: #d7e1ea;
    }
    /* Visual-Studio-like tool-window tabs at the top of each dock group. */
    QTabBar::tab {
      background: #141c25;
      color: #9fb0c0;
      border: 1px solid #2b3947;
      border-bottom: none;
      padding: 6px 14px;
      margin-right: 2px;
    }
    QTabBar::tab:selected {
      background: #1d2937;
      color: #e6ecf2;
      border-top: 2px solid #2f8cff;
    }
    QTabBar::tab:hover {
      color: #e6ecf2;
    }
    /* Teach-pendant look: a dark bezel with an LCD read-out. */
    QWidget#pendant {
      background: #0d1117;
      border: 1px solid #2b3947;
      border-radius: 8px;
    }
    QLabel#pendantTitle {
      color: #6f8296;
      font-weight: bold;
      letter-spacing: 3px;
      padding: 2px 0;
    }
    QLabel#pendantLcd {
      background: #06120e;
      color: #46f0a0;                 /* LCD green */
      border: 1px solid #123326;
      border-radius: 4px;
      padding: 8px;
      font-family: "Menlo", "Consolas", monospace;
      font-size: 13px;
    }
    QWidget#pendant QPushButton {
      background: #1a2430;
      border: 1px solid #33465a;
      border-radius: 5px;
      padding: 7px 10px;
      font-weight: bold;
    }
    QWidget#pendant QPushButton:hover { border-color: #4d9dff; }
    QWidget#pendant QPushButton:pressed { background: #223247; }
  )qss");
}
