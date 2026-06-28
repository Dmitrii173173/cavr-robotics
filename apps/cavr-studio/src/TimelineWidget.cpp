#include "TimelineWidget.hpp"

#include <QPainter>
#include <QPaintEvent>

TimelineWidget::TimelineWidget(QWidget* parent) : QWidget(parent) {
  setMinimumHeight(220);
  connect(&timer_, &QTimer::timeout, this, [this] {
    playhead_ += 0.0025;
    if (playhead_ > 0.72) {
      playhead_ = 0.48;
    }
    update();
  });
  timer_.start(33);
}

void TimelineWidget::paintEvent(QPaintEvent* event) {
  Q_UNUSED(event);
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.fillRect(rect(), QColor("#18212b"));

  const int label_width = 132;
  const int top = 38;
  const int track_height = 27;
  const QStringList labels = {"/robot/state", "/robot/joints", "/camera/rgb", "/camera/depth", "/camera/pointcloud", "/session/event"};
  const QList<QColor> colors = {QColor("#ff4d45"), QColor("#ff9f1a"), QColor("#55d66b"), QColor("#2f8cff"), QColor("#b8ac80"), QColor("#f3c84b")};

  painter.setPen(QColor("#9aa9b8"));
  painter.drawText(12, 24, "00:01:23.456 / 00:02:34.893");

  const QRect ruler(label_width, 12, width() - label_width - 20, 18);
  for (int tick = 0; tick <= 5; ++tick) {
    const int x = ruler.left() + tick * ruler.width() / 5;
    painter.drawLine(x, ruler.bottom(), x, height() - 14);
    painter.drawText(x - 24, ruler.top(), 60, 14, Qt::AlignCenter, QString("00:0%1:00").arg(tick));
  }

  for (int row = 0; row < labels.size(); ++row) {
    const int y = top + row * track_height;
    painter.setPen(QColor("#cbd6e2"));
    painter.drawText(12, y + 18, labels[row]);
    QRect track(label_width, y + 6, width() - label_width - 20, 10);
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor("#111922"));
    painter.drawRoundedRect(track, 5, 5);
    painter.setBrush(colors[row]);
    for (int mark = 0; mark < 18; ++mark) {
      const int x = track.left() + 14 + mark * track.width() / 18;
      painter.drawEllipse(QPointF(x, track.center().y()), row < 2 ? 3 : 5, row < 2 ? 3 : 5);
    }
  }

  const int playhead_x = label_width + static_cast<int>((width() - label_width - 20) * playhead_);
  painter.setPen(QPen(QColor("#2f8cff"), 2));
  painter.drawLine(playhead_x, 30, playhead_x, height() - 12);
  painter.setBrush(QColor("#2f8cff"));
  QPolygon marker;
  marker << QPoint(playhead_x - 7, 30) << QPoint(playhead_x + 7, 30) << QPoint(playhead_x, 39);
  painter.drawPolygon(marker);
}
