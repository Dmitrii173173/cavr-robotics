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
#include <QSignalBlocker>
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
  // Registry changes (save/load/delete) repopulate the robot list + IO table, and
  // re-validate the program against the (possibly new) robot profile.
  connect(controller_, &RobotController::robotsChanged, this, [this] {
    refresh_robots();
    refresh_program();
  });
  // Editing the program repopulates the step list; DB changes the saved-jobs list.
  connect(controller_, &RobotController::programChanged, this, [this] { refresh_program(); });
  connect(controller_, &RobotController::programSelectionChanged, this, [this] { refresh_program(); });
  connect(controller_, &RobotController::savedProgramsChanged, this,
          [this] { refresh_saved_programs(); });
  // Each telemetry tick refreshes the live IO values, joint limits, pendant and the
  // live vision-guidance read-out.
  connect(controller_, &RobotController::telemetryChanged, this, [this] {
    refresh_io();
    refresh_joints();
    update_pendant();
    refresh_calibration();
  });
  refresh_robots();
  refresh_program();
  refresh_saved_programs();
  refresh_joints();
  update_pendant();
  refresh_calibration();
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
  // Live jog (scene -> robot): commands the robot вЂ” in-process mock or, with
  // CAVR_ROBOT_ENDPOINT set, a remote one вЂ” to move home right now.
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
  auto* timeline = make_dock("Timeline", new TimelineWidget(controller_, this));

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

  auto* save = new QPushButton("Save current asвЂ¦");
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
    const QString label = QString("%1  вЂ”  %2  (%3 axes, %4 IO)")
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
  connect(program_list_, &QListWidget::currentRowChanged, this,
          [this](int row) { controller_->selectProgramStep(row); });

  // Pre-flight validation summary (green when valid, red on errors).
  validation_label_ = new QLabel;
  validation_label_->setObjectName("validationSummary");
  layout->addWidget(validation_label_);

  // Collision status (self-collision + floor over the sampled trajectory).
  collision_label_ = new QLabel;
  collision_label_->setObjectName("collisionSummary");
  layout->addWidget(collision_label_);

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
  auto* up = new QPushButton("в†‘");
  auto* down = new QPushButton("в†“");
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

  auto* run = new QPushButton("в–¶ Run program");
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
  const QSignalBlocker blocker(program_list_);
  program_list_->clear();
  for (const QVariant& entry : controller_->programSteps()) {
    const QVariantMap m = entry.toMap();
    const int status = m.value("status").toInt();  // 0 ok, 1 warn, 2 error (validation)
    QString prefix = m.value("active").toBool() ? "в–¶ " : "  ";
    if (m.value("selected").toBool()) prefix = "в—Џ ";
    auto* item = new QListWidgetItem(
        prefix + QString("%1.  %2   %3")
                     .arg(m.value("index").toInt(), 2)
                     .arg(m.value("kind").toString().toUpper(), -8)
                     .arg(m.value("detail").toString()),
        program_list_);
    // Validation status colouring: red = error, amber = warning, green = ok.
    if (status == 2) item->setForeground(QColor("#ff6b6b"));
    else if (status == 1) item->setForeground(QColor("#f0b866"));
    else item->setForeground(QColor("#46f0a0"));
    if (m.value("selected").toBool()) program_list_->setCurrentItem(item);
  }
  if (validation_label_) {
    validation_label_->setText(controller_->validationSummary());
    const bool ok = controller_->programValid();
    validation_label_->setStyleSheet(ok ? "color:#46f0a0;" : "color:#ff6b6b; font-weight:bold;");
  }
  if (collision_label_) {
    const QString summary = controller_->collisionSummary();
    collision_label_->setText("Collisions: " + summary);
    const bool clean = summary.contains("no collision") || summary.contains("no steps");
    collision_label_->setStyleSheet(clean ? "color:#46f0a0;" : "color:#ff6b6b; font-weight:bold;");
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

void StudioWindow::refresh_joints() {
  if (!joint_table_ || !controller_) return;
  const QVariantList joints = controller_->jointStatus();
  joint_table_->setRowCount(static_cast<int>(joints.size()));
  for (int r = 0; r < joints.size(); ++r) {
    const QVariantMap j = joints[r].toMap();
    const bool over = j.value("over").toBool();
    const auto cell = [&](const QString& text, bool highlight) {
      auto* it = new QTableWidgetItem(text);
      if (highlight) it->setForeground(QColor("#ff6b6b"));
      return it;
    };
    joint_table_->setItem(r, 0, cell(j.value("name").toString(), over));
    joint_table_->setItem(r, 1, cell(QString::number(j.value("value").toDouble(), 'f', 1), over));
    joint_table_->setItem(r, 2, cell(QString::number(j.value("lower").toDouble(), 'f', 0), false));
    joint_table_->setItem(r, 3, cell(QString::number(j.value("upper").toDouble(), 'f', 0), false));
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
  auto* layout = new QVBoxLayout(panel);

  auto* form = new QFormLayout;
  form->addRow("Camera Intrinsics", value_label("cam_01_intrinsics.yaml"));
  form->addRow("Hand-Eye", value_label("he_2025_05_10.yaml"));
  form->addRow("Reprojection Error", value_label("0.42 px"));
  form->addRow("Status", value_label("Valid"));
  layout->addLayout(form);

  layout->addWidget(horizontal_rule());

  // Vision-guided seam correction: the live scan is placed in the base frame via
  // the hand-eye extrinsics and compared to the planned seam. The read-out updates
  // each telemetry tick; Apply shifts the program's Cartesian targets by the offset.
  auto* vision_title = new QLabel("Vision guidance");
  vision_title->setStyleSheet("font-weight:bold;");
  layout->addWidget(vision_title);

  vision_label_ = new QLabel("no live scan — press Run Demo");
  vision_label_->setObjectName("visionSummary");
  vision_label_->setWordWrap(true);
  layout->addWidget(vision_label_);

  auto* apply_seam = new QPushButton("Apply seam correction");
  connect(apply_seam, &QPushButton::clicked, this, [this] {
    controller_->applySeamCorrection();
    refresh_calibration();
  });
  layout->addWidget(apply_seam);

  layout->addStretch();
  return panel;
}

void StudioWindow::refresh_calibration() {
  if (!controller_ || !vision_label_) return;
  vision_label_->setText(controller_->visionSummary());
  vision_label_->setStyleSheet(controller_->hasScan() ? "color:#7fd0ff;" : "color:#8b96a0;");
}

QWidget* StudioWindow::make_jog_panel() {
  {
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

    auto* panel = new QWidget;
    panel->setObjectName("pendant");
    auto* layout = new QVBoxLayout(panel);
    layout->setContentsMargins(10, 10, 10, 10);

    const auto add_section = [&](const QString& text) {
      auto* label = new QLabel(text);
      label->setObjectName("sectionLabel");
      layout->addWidget(label);
    };

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

    // Joint limits: each axis's live position against its configured range. A joint
    // at or past its limit turns red — a limit-error surfaced right in the panel.
    layout->addWidget(new QLabel("Joints (position / limits, °)"));
    joint_table_ = new QTableWidget(0, 4);
    joint_table_->setHorizontalHeaderLabels({"Axis", "Pos", "Min", "Max"});
    joint_table_->verticalHeader()->hide();
    joint_table_->horizontalHeader()->setStretchLastSection(true);
    joint_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    joint_table_->setSelectionMode(QAbstractItemView::NoSelection);
    joint_table_->setMaximumHeight(190);
    layout->addWidget(joint_table_);

    layout->addWidget(horizontal_rule());
    add_section("1. LIVE MANUAL JOG");
    auto* mode_hint = new QLabel("Manual moves go to the robot now. Use low step sizes near fixtures.");
    mode_hint->setObjectName("hintLabel");
    mode_hint->setWordWrap(true);
    layout->addWidget(mode_hint);

    auto* frame_speed = new QGridLayout;
    frame_speed->addWidget(new QLabel("Frame"), 0, 0);
    auto* frame = new QComboBox;
    frame->addItems({"World", "Base", "Tool", "User"});
    frame->setCurrentIndex(1);
    connect(frame, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            [this](int i) { controller_->setCoordinateSystem(i); });
    frame_speed->addWidget(frame, 0, 1);
    frame_speed->addWidget(new QLabel("Speed"), 0, 2);
    auto* speed = new QDoubleSpinBox;
    speed->setRange(1.0, 2000.0);
    speed->setValue(50.0);
    speed->setSuffix(" mm/s");
    connect(speed, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            [this](double v) { controller_->setSpeedMmS(v); });
    frame_speed->addWidget(speed, 0, 3);
    layout->addLayout(frame_speed);

    auto* safety_row = new QHBoxLayout;
    auto* home = new QPushButton("Home");
    auto* pause = new QPushButton("Hold");
    auto* resume = new QPushButton("Resume");
    auto* stop = new QPushButton("Stop");
    home->setObjectName("primaryButton");
    stop->setObjectName("dangerButton");
    connect(home, &QPushButton::clicked, this, [this] { controller_->jogHome(); });
    connect(pause, &QPushButton::clicked, this, [this] { controller_->pause(); });
    connect(resume, &QPushButton::clicked, this, [this] { controller_->resume(); });
    connect(stop, &QPushButton::clicked, this, [this] { controller_->stop(); });
    for (auto* b : {home, pause, resume, stop}) safety_row->addWidget(b);
    layout->addLayout(safety_row);

    auto* step_grid = new QGridLayout;
    step_grid->addWidget(new QLabel("Joint step"), 0, 0);
    auto* joint_step = new QComboBox;
    for (double d : {0.1, 1.0, 5.0, 10.0}) joint_step->addItem(QString("%1 deg").arg(d), d);
    joint_step->setCurrentIndex(2);
    step_grid->addWidget(joint_step, 0, 1);
    step_grid->addWidget(new QLabel("Linear step"), 0, 2);
    auto* linear_step = new QComboBox;
    for (double d : {1.0, 5.0, 10.0, 50.0}) linear_step->addItem(QString("%1 mm").arg(d), d);
    linear_step->setCurrentIndex(3);
    step_grid->addWidget(linear_step, 0, 3);
    step_grid->addWidget(new QLabel("Rot step"), 1, 0);
    auto* rot_step = new QComboBox;
    for (double d : {0.5, 1.0, 5.0, 15.0}) rot_step->addItem(QString("%1 deg").arg(d), d);
    rot_step->setCurrentIndex(2);
    step_grid->addWidget(rot_step, 1, 1);
    layout->addLayout(step_grid);

    add_section("Joint jog");
    const QStringList axes = {"S", "L", "U", "R", "B", "T"};
    for (int i = 0; i < axes.size(); ++i) {
      auto* row = new QHBoxLayout;
      auto* axis = new QLabel(axes[i]);
      axis->setMinimumWidth(28);
      row->addWidget(axis);
      auto* minus = new QPushButton("-");
      auto* plus = new QPushButton("+");
      connect(minus, &QPushButton::clicked, this,
              [this, i, joint_step] { controller_->jogJoint(i, -joint_step->currentData().toDouble()); });
      connect(plus, &QPushButton::clicked, this,
              [this, i, joint_step] { controller_->jogJoint(i, joint_step->currentData().toDouble()); });
      row->addWidget(minus);
      row->addWidget(plus);
      layout->addLayout(row);
    }

    add_section("Cartesian jog");
    const struct {
      const char* label;
      int component;
    } cart[] = {{"X", 0}, {"Y", 1}, {"Z", 2}, {"Rx", 3}, {"Ry", 4}, {"Rz", 5}};
    for (const auto& c : cart) {
      auto* row = new QHBoxLayout;
      auto* label = new QLabel(c.label);
      label->setMinimumWidth(28);
      row->addWidget(label);
      auto* minus = new QPushButton("-");
      auto* plus = new QPushButton("+");
      const int component = c.component;
      const auto jog_cart = [this, component, linear_step, rot_step, kDegToRad](double sign) {
        double tx = 0, ty = 0, tz = 0, rx = 0, ry = 0, rz = 0;
        if (component < 3) {
          const double m = sign * linear_step->currentData().toDouble() / 1000.0;
          if (component == 0) tx = m;
          if (component == 1) ty = m;
          if (component == 2) tz = m;
        } else {
          const double r = sign * rot_step->currentData().toDouble() * kDegToRad;
          if (component == 3) rx = r;
          if (component == 4) ry = r;
          if (component == 5) rz = r;
        }
        controller_->jogCartesian(tx, ty, tz, rx, ry, rz);
      };
      connect(minus, &QPushButton::clicked, this, [jog_cart] { jog_cart(-1.0); });
      connect(plus, &QPushButton::clicked, this, [jog_cart] { jog_cart(1.0); });
      row->addWidget(minus);
      row->addWidget(plus);
      layout->addLayout(row);
    }

    layout->addWidget(horizontal_rule());
    add_section("2. TEACH TO TIMELINE");
    auto* teach_hint = new QLabel("Teach reads the current robot pose and adds a program block. It does not move the robot.");
    teach_hint->setObjectName("hintLabel");
    teach_hint->setWordWrap(true);
    layout->addWidget(teach_hint);

    auto* teach_grid = new QGridLayout;
    const struct {
      const char* label;
      const char* object;
      std::function<void()> action;
    } teach[] = {
        {"Teach MoveJ", "teachButton", [this] { controller_->addMoveJ(); }},
        {"Teach MoveL", "teachButton", [this] { controller_->addMoveL(); }},
        {"Set Arc Via", "", [this] { controller_->setVia(); }},
        {"Teach MoveC End", "teachButton", [this] { controller_->addMoveC(); }},
        {"Tool On", "", [this] { controller_->addToolOn(); }},
        {"Tool Off", "", [this] { controller_->addToolOff(); }},
    };
    const int teach_count = static_cast<int>(sizeof(teach) / sizeof(teach[0]));
    for (int i = 0; i < teach_count; ++i) {
      auto* b = new QPushButton(teach[i].label);
      if (*teach[i].object) b->setObjectName(teach[i].object);
      const auto action = teach[i].action;
      connect(b, &QPushButton::clicked, this, [action] { action(); });
      teach_grid->addWidget(b, i / 2, i % 2);
    }
    layout->addLayout(teach_grid);

    auto* wait_row = new QHBoxLayout;
    auto* wait = new QDoubleSpinBox;
    wait->setRange(0.0, 60.0);
    wait->setValue(1.0);
    wait->setSuffix(" s");
    auto* add_wait = new QPushButton("Teach Wait");
    connect(add_wait, &QPushButton::clicked, this,
            [this, wait] { controller_->addWait(wait->value()); });
    wait_row->addWidget(wait);
    wait_row->addWidget(add_wait);
    layout->addLayout(wait_row);

    auto* run_row = new QHBoxLayout;
    auto* run_program = new QPushButton("Run Timeline");
    run_program->setObjectName("primaryButton");
    auto* clear_program = new QPushButton("Clear Timeline");
    connect(run_program, &QPushButton::clicked, this, [this] { controller_->runProgram(); });
    connect(clear_program, &QPushButton::clicked, this, [this] { controller_->clearProgram(); });
    run_row->addWidget(run_program);
    run_row->addWidget(clear_program);
    layout->addLayout(run_row);

    layout->addWidget(horizontal_rule());
    add_section("3. TOOL TABLE / TCP");
    auto* tool_row = new QHBoxLayout;
    tool_row->addWidget(new QLabel("Active slot"));
    auto* tool = new QComboBox;
    for (int i = 0; i < 10; ++i) tool->addItem(QString::number(i));
    connect(tool, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            [this](int i) { controller_->selectTool(i); });
    tool_row->addWidget(tool);
    layout->addLayout(tool_row);

    auto* cal_row = new QHBoxLayout;
    cal_row->addWidget(new QLabel("TCP m"));
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
    auto* calibrate = new QPushButton("Calibrate TCP");
    auto* clear_tool = new QPushButton("Clear Slot");
    connect(calibrate, &QPushButton::clicked, this, [this, tool, tcp_x, tcp_y, tcp_z] {
      controller_->calibrateTool(tool->currentIndex(), tcp_x->value(), tcp_y->value(), tcp_z->value());
    });
    connect(clear_tool, &QPushButton::clicked, this,
            [this, tool] { controller_->clearTool(tool->currentIndex()); });
    tool_btns->addWidget(calibrate);
    tool_btns->addWidget(clear_tool);
    layout->addLayout(tool_btns);

    auto* demo = new QPushButton("Run Demo Cycle");
    connect(demo, &QPushButton::clicked, this, [this] { controller_->runDemo(); });
    layout->addWidget(demo);
    layout->addStretch();

    auto* scroll = new QScrollArea;
    scroll->setWidget(panel);
    scroll->setWidgetResizable(true);
    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    return scroll;
  }
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
    QLabel#sectionLabel {
      color: #f2f7ff;
      font-weight: 700;
      padding: 8px 0 3px 0;
      border-top: 1px solid #243241;
    }
    QLabel#hintLabel {
      color: #8fa4b8;
      font-size: 12px;
      padding: 0 0 4px 0;
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
    QWidget#pendant QPushButton#primaryButton {
      background: #1167d8;
      border-color: #5aa8ff;
      color: #ffffff;
    }
    QWidget#pendant QPushButton#teachButton {
      background: #0f5a3a;
      border-color: #38d07f;
      color: #eafff3;
    }
    QWidget#pendant QPushButton#dangerButton {
      background: #5c1f24;
      border-color: #ff5a66;
      color: #ffe8ea;
    }
  )qss");
}
