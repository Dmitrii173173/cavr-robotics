#include "StudioWindow.hpp"

#include "CameraView.hpp"
#include "RobotController.hpp"
#include "TimelineWidget.hpp"

#include <QAction>
#include <QQmlContext>
#include <QQuickWidget>
#include <QUrl>
#include <QDockWidget>
#include <QFormLayout>
#include <QFrame>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QSplitter>
#include <QStatusBar>
#include <QTableWidget>
#include <QToolBar>
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
  auto* session = make_dock("1 Session", make_session_panel());
  auto* channels = make_dock("2 Channels", make_channels_panel());
  auto* events = make_dock("3 Events", make_events_panel());
  auto* camera = make_dock("5 Camera View", new CameraView(this));
  auto* telemetry = make_dock("6 Telemetry", make_telemetry_panel());
  auto* timeline = make_dock("7 Timeline", new TimelineWidget(this));
  auto* calibration = make_dock("8 Calibration", make_calibration_panel());
  auto* faults = make_dock("9 Fault Injection", make_fault_panel());

  addDockWidget(Qt::LeftDockWidgetArea, session);
  splitDockWidget(session, channels, Qt::Vertical);
  splitDockWidget(channels, events, Qt::Vertical);

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
