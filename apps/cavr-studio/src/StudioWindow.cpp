#include "StudioWindow.hpp"

#include "CameraView.hpp"
#include "RobotController.hpp"
#include "TimelineWidget.hpp"

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
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QSplitter>
#include <QStatusBar>
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
  // Each telemetry tick refreshes the live IO values.
  connect(controller_, &RobotController::telemetryChanged, this, [this] { refresh_io(); });
  refresh_robots();
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
  auto* robots = make_dock("1 Robots", make_robots_panel());
  auto* session = make_dock("2 Session", make_session_panel());
  auto* channels = make_dock("3 Channels", make_channels_panel());
  auto* events = make_dock("4 Events", make_events_panel());
  auto* jog = make_dock("5 Jog", make_jog_panel());
  auto* camera = make_dock("6 Camera View", new CameraView(this));
  auto* telemetry = make_dock("7 Telemetry", make_telemetry_panel());
  auto* timeline = make_dock("8 Timeline", new TimelineWidget(this));
  auto* calibration = make_dock("9 Calibration", make_calibration_panel());
  auto* faults = make_dock("10 Fault Injection", make_fault_panel());

  addDockWidget(Qt::LeftDockWidgetArea, robots);
  splitDockWidget(robots, session, Qt::Vertical);
  splitDockWidget(session, channels, Qt::Vertical);
  splitDockWidget(channels, events, Qt::Vertical);
  splitDockWidget(events, jog, Qt::Vertical);

  addDockWidget(Qt::RightDockWidgetArea, camera);
  splitDockWidget(camera, telemetry, Qt::Vertical);
  splitDockWidget(telemetry, calibration, Qt::Vertical);
  splitDockWidget(calibration, faults, Qt::Vertical);

  addDockWidget(Qt::BottomDockWidgetArea, timeline);
  timeline->setMinimumHeight(240);
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
  auto* layout = new QVBoxLayout(panel);

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
  auto* demo = new QPushButton("Run Demo");
  connect(home, &QPushButton::clicked, this, [this] { controller_->jogHome(); });
  connect(demo, &QPushButton::clicked, this, [this] { controller_->runDemo(); });
  layout->addWidget(home);
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
  )qss");
}
