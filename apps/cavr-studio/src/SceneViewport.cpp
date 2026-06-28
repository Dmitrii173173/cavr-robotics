#include "SceneViewport.hpp"

#include <cavr/visualization/scene_model.hpp>

#include <QBrush>
#include <QLinearGradient>
#include <QPainter>
#include <QPainterPath>
#include <QPen>
#include <QtMath>

namespace {

constexpr double pi = 3.14159265358979323846;

QColor color_from_rgb(const cavr::visualization::ColorRgb& color) {
  return QColor::fromRgbF(color.r, color.g, color.b);
}

}  // namespace

SceneViewport::SceneViewport(QWidget* parent) : QOpenGLWidget(parent) {
  setMinimumSize(520, 380);
  setAutoFillBackground(false);
  connect(&timer_, &QTimer::timeout, this, [this] {
    phase_ += 0.025;
    update();
  });
  timer_.start(16);
}

void SceneViewport::resizeGL(int width, int height) {
  Q_UNUSED(width);
  Q_UNUSED(height);
}

void SceneViewport::paintGL() {
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, true);

  QLinearGradient background(rect().topLeft(), rect().bottomRight());
  background.setColorAt(0.0, QColor("#111923"));
  background.setColorAt(1.0, QColor("#172431"));
  painter.fillRect(rect(), background);

  draw_grid(painter);
  draw_workpiece(painter);
  draw_point_cloud(painter);
  draw_trajectory(painter);
  draw_robot(painter);
  draw_axes(painter);
  draw_transform_legend(painter);
}

QPointF SceneViewport::project(double x, double y, double z) const {
  const double scale = 1.0 / (1.0 + z * 0.0032);
  return {width() * 0.52 + (x - z * 0.38) * scale,
          height() * 0.58 + (y + z * 0.18) * scale};
}

void SceneViewport::draw_grid(QPainter& painter) {
  painter.setPen(QPen(QColor(91, 114, 136, 72), 1));
  for (int index = -12; index <= 12; ++index) {
    painter.drawLine(project(index * 42.0, 0.0, -360.0), project(index * 42.0, 0.0, 360.0));
  }
  for (int z = -360; z <= 360; z += 42) {
    painter.drawLine(project(-520.0, 0.0, z), project(520.0, 0.0, z));
  }
}

void SceneViewport::draw_workpiece(QPainter& painter) {
  QPolygonF plate;
  plate << project(70, 34, -120) << project(340, 34, -70) << project(260, 34, 190) << project(-10, 34, 135);
  painter.setPen(QPen(QColor("#6a747d"), 1));
  painter.setBrush(QColor("#343c42"));
  painter.drawPolygon(plate);

  painter.setPen(QPen(QColor("#bdb9a8"), 5, Qt::SolidLine, Qt::RoundCap));
  painter.drawLine(project(45, 20, 0), project(290, 20, 42));
}

void SceneViewport::draw_point_cloud(QPainter& painter) {
  quint32 seed = 42;
  for (int index = 0; index < 620; ++index) {
    seed = seed * 1664525u + 1013904223u;
    const double rx = static_cast<double>(seed) / static_cast<double>(std::numeric_limits<quint32>::max()) - 0.5;
    seed = seed * 1664525u + 1013904223u;
    const double rz = static_cast<double>(seed) / static_cast<double>(std::numeric_limits<quint32>::max()) - 0.5;
    const QPointF point = project(155 + rx * 330, 10 + rz * 24, 35 + rz * 260);
    painter.setPen(index % 7 == 0 ? QColor(255, 191, 71, 210) : QColor(220, 228, 232, 184));
    painter.drawPoint(point);
  }
}

void SceneViewport::draw_trajectory(QPainter& painter) {
  painter.setPen(QPen(QColor("#ff4d45"), 2));
  QPainterPath path;
  for (int index = 0; index <= 72; ++index) {
    const double angle = (static_cast<double>(index) / 72.0) * pi * 2.0 + phase_ * 0.4;
    const QPointF point = project(175 + std::cos(angle) * 92, -72 + std::sin(angle) * 70, 42);
    index == 0 ? path.moveTo(point) : path.lineTo(point);
  }
  painter.drawPath(path);

  painter.setBrush(QColor("#ff4d45"));
  for (int index = 0; index < 18; ++index) {
    const double angle = (static_cast<double>(index) / 18.0) * pi * 2.0 + phase_ * 0.4;
    painter.drawEllipse(project(175 + std::cos(angle) * 92, -72 + std::sin(angle) * 70, 42), 3, 3);
  }
}

void SceneViewport::draw_robot(QPainter& painter) {
  const QPointF base = project(-330, 30, -80);
  const QPointF shoulder = project(-290, -84, -80);
  const QPointF elbow = project(-150, -144 + std::sin(phase_) * 12, -30);
  const QPointF wrist = project(16, -92 + std::cos(phase_) * 10, 22);
  const QPointF tcp = project(74, -18, 40);

  painter.setPen(Qt::NoPen);
  painter.setBrush(QColor("#70777e"));
  painter.drawEllipse(QRectF(base.x() - 62, base.y() + 1, 124, 42));
  painter.setBrush(QColor("#8a9097"));
  painter.drawRoundedRect(QRectF(base.x() - 46, base.y() - 8, 92, 32), 4, 4);

  const auto draw_segment = [&painter](const QPointF& from, const QPointF& to, int width, const QColor& color) {
    painter.setPen(QPen(color, width, Qt::SolidLine, Qt::RoundCap));
    painter.drawLine(from, to);
    painter.setPen(QPen(QColor(255, 255, 255, 64), 1));
    painter.drawLine(from, to);
  };

  draw_segment(base, shoulder, 34, QColor("#838990"));
  draw_segment(shoulder, elbow, 34, QColor("#9ca1a6"));
  draw_segment(elbow, wrist, 28, QColor("#a6abb0"));
  draw_segment(wrist, tcp, 16, QColor("#b07f3b"));

  painter.setPen(QPen(QColor("#c6cdd4"), 1));
  painter.setBrush(QColor("#8d969f"));
  for (const QPointF& point : {base, shoulder, elbow, wrist}) {
    painter.drawEllipse(point, 18, 18);
  }
  painter.setPen(Qt::NoPen);
  painter.setBrush(QColor("#f3a23a"));
  painter.drawEllipse(tcp, 7, 7);
}

void SceneViewport::draw_axes(QPainter& painter) {
  const QPointF origin(48, height() - 64);
  const QVector<QPair<QString, QLineF>> axes = {
      {"X", QLineF(origin, origin + QPointF(48, 20))},
      {"Y", QLineF(origin, origin + QPointF(30, -38))},
      {"Z", QLineF(origin, origin + QPointF(0, -58))},
  };
  const QVector<QColor> colors = {QColor("#ff4d45"), QColor("#55d66b"), QColor("#2f8cff")};
  for (int index = 0; index < axes.size(); ++index) {
    painter.setPen(QPen(colors[index], 2));
    painter.drawLine(axes[index].second);
    painter.drawText(axes[index].second.p2() + QPointF(5, 4), axes[index].first);
  }
}

void SceneViewport::draw_transform_legend(QPainter& painter) {
  const QRectF panel(width() - 156, 70, 136, 148);
  painter.setPen(QPen(QColor("#405064"), 1));
  painter.setBrush(QColor(18, 25, 33, 220));
  painter.drawRoundedRect(panel, 6, 6);
  painter.setPen(QColor("#e6ecf2"));
  painter.drawText(panel.adjusted(10, 10, -10, -10), "Transforms");

  int row = 0;
  for (const auto& frame : cavr::visualization::default_frame_tree()) {
    const int y = static_cast<int>(panel.top()) + 34 + row * 18;
    painter.setBrush(color_from_rgb(frame.color));
    painter.setPen(Qt::NoPen);
    painter.drawEllipse(QPointF(panel.right() - 18, y - 4), 4, 4);
    painter.setPen(QColor("#d7e1ea"));
    painter.drawText(QPointF(panel.left() + 12, y), QString::fromUtf8(frame.name.data(), static_cast<qsizetype>(frame.name.size())));
    ++row;
  }
}
