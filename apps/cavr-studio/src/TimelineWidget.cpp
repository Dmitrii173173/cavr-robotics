#include "TimelineWidget.hpp"

#include "RobotController.hpp"

#include <QFontMetrics>
#include <QMouseEvent>
#include <QPainter>
#include <QPaintEvent>
#include <QVariantMap>

#include <algorithm>

namespace {

QColor color_for_kind(const QString& kind) {
  if (kind == "movej") return QColor("#2f8cff");
  if (kind == "movel") return QColor("#29c782");
  if (kind == "movec") return QColor("#b45cff");
  if (kind == "tool_on") return QColor("#ffb020");
  if (kind == "tool_off") return QColor("#ff6b4a");
  if (kind == "wait") return QColor("#8aa1b5");
  return QColor("#9aa9b8");
}

int lane_for_kind(const QString& kind) {
  if (kind == "tool_on" || kind == "tool_off") return 1;
  if (kind == "wait") return 2;
  return 0;
}

QString lane_name(int lane) {
  switch (lane) {
    case 0: return "Robot motion";
    case 1: return "Process / tool";
    case 2: return "Wait / IO";
    default: return {};
  }
}

}  // namespace

TimelineWidget::TimelineWidget(RobotController* controller, QWidget* parent)
    : QWidget(parent), controller_(controller) {
  setMinimumHeight(220);
  setMouseTracking(true);
  if (controller_) {
    connect(controller_, &RobotController::programChanged, this, [this] { update(); });
    connect(controller_, &RobotController::programSelectionChanged, this, [this] { update(); });
    connect(controller_, &RobotController::telemetryChanged, this, [this] { update(); });
  }
}

void TimelineWidget::paintEvent(QPaintEvent* event) {
  Q_UNUSED(event);
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.fillRect(rect(), QColor("#18212b"));
  hit_blocks_.clear();

  const QVariantList steps = controller_ ? controller_->programSteps() : QVariantList{};
  painter.setPen(QColor("#d7e1ea"));
  QFont title = painter.font();
  title.setBold(true);
  painter.setFont(title);
  painter.drawText(14, 24, "Program Timeline");
  painter.setFont(QFont());
  painter.setPen(QColor("#8da1b4"));
  painter.drawText(150, 24, "timeline is the job; Jog/Teach adds real robot commands here");

  if (steps.empty()) {
    QRect empty(18, 58, width() - 36, height() - 82);
    painter.setPen(QPen(QColor("#344557"), 1, Qt::DashLine));
    painter.setBrush(QColor("#111922"));
    painter.drawRoundedRect(empty, 8, 8);
    painter.setPen(QColor("#9fb0c0"));
    painter.drawText(empty, Qt::AlignCenter,
                     "No program steps yet\nUse Teach MoveJ / MoveL / MoveC from Jog + Tools.");
    return;
  }

  const int label_width = 128;
  const int left = label_width + 18;
  const int right = width() - 18;
  const int top = 52;
  const int lane_height = 45;
  const int block_height = 26;
  const int gap = 7;
  const int lane_count = 3;
  const int usable = std::max(120, right - left);

  double total = 0.0;
  for (const QVariant& v : steps) total += std::max(0.2, v.toMap().value("duration").toDouble());

  painter.setPen(QColor("#405267"));
  for (int tick = 0; tick <= 4; ++tick) {
    const int x = left + tick * usable / 4;
    painter.drawLine(x, top - 16, x, top + lane_count * lane_height + 4);
    painter.setPen(QColor("#74879b"));
    painter.drawText(x - 18, top - 22, 42, 14, Qt::AlignCenter,
                     QString("%1%").arg(tick * 25));
    painter.setPen(QColor("#405267"));
  }

  for (int lane = 0; lane < lane_count; ++lane) {
    const int y = top + lane * lane_height;
    painter.setPen(QColor("#cbd6e2"));
    painter.drawText(14, y + 24, lane_name(lane));
    QRect track(left, y + 12, usable, block_height);
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor("#111922"));
    painter.drawRoundedRect(track, 6, 6);
  }

  double cursor = 0.0;
  int active_x = -1;
  QFontMetrics fm(painter.font());
  for (const QVariant& v : steps) {
    const QVariantMap m = v.toMap();
    const QString kind = m.value("kind").toString();
    const int lane = lane_for_kind(kind);
    const double dur = std::max(0.2, m.value("duration").toDouble());
    const int x = left + static_cast<int>((cursor / total) * usable);
    const int w = std::max(46, static_cast<int>((dur / total) * usable) - gap);
    const int y = top + lane * lane_height + 12;
    QRect block(x, y, std::min(w, right - x), block_height);

    const bool selected = m.value("selected").toBool();
    const bool active = m.value("active").toBool();
    QColor fill = color_for_kind(kind);
    if (!active) fill = fill.darker(selected ? 100 : 125);
    painter.setBrush(fill);
    painter.setPen(QPen(selected ? QColor("#ffffff") : QColor("#263544"), selected ? 2 : 1));
    painter.drawRoundedRect(block, 6, 6);

    painter.setPen(QColor("#f7fbff"));
    const QString label = QString("%1 %2").arg(m.value("index").toInt()).arg(kind.toUpper());
    painter.drawText(block.adjusted(8, 0, -6, 0), Qt::AlignVCenter | Qt::AlignLeft,
                     fm.elidedText(label, Qt::ElideRight, block.width() - 14));

    hit_blocks_.push_back({block.adjusted(-2, -4, 2, 4), m.value("zeroIndex").toInt()});
    if (active) active_x = block.center().x();
    cursor += dur;
  }

  if (active_x >= 0) {
    painter.setPen(QPen(QColor("#ffffff"), 2));
    painter.drawLine(active_x, top - 10, active_x, top + lane_count * lane_height + 10);
    painter.setBrush(QColor("#ffffff"));
    QPolygon marker;
    marker << QPoint(active_x - 7, top - 10) << QPoint(active_x + 7, top - 10) << QPoint(active_x, top);
    painter.drawPolygon(marker);
  }
}

void TimelineWidget::mousePressEvent(QMouseEvent* event) {
  if (!controller_) return;
  for (const auto& block : hit_blocks_) {
    if (block.first.contains(event->pos())) {
      controller_->selectProgramStep(block.second);
      return;
    }
  }
  controller_->selectProgramStep(-1);
}
