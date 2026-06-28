#pragma once

#include <QOpenGLWidget>
#include <QPointF>
#include <QTimer>

class SceneViewport final : public QOpenGLWidget {
 public:
  explicit SceneViewport(QWidget* parent = nullptr);

 protected:
  void paintGL() override;
  void resizeGL(int width, int height) override;

 private:
  [[nodiscard]] QPointF project(double x, double y, double z) const;
  void draw_grid(QPainter& painter);
  void draw_robot(QPainter& painter);
  void draw_point_cloud(QPainter& painter);
  void draw_workpiece(QPainter& painter);
  void draw_trajectory(QPainter& painter);
  void draw_axes(QPainter& painter);
  void draw_transform_legend(QPainter& painter);

  QTimer timer_;
  double phase_{};
};
