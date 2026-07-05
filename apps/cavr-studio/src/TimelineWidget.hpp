#pragma once

#include <QRect>
#include <QVector>
#include <QWidget>

class RobotController;

class TimelineWidget final : public QWidget {
 public:
  explicit TimelineWidget(RobotController* controller, QWidget* parent = nullptr);

 protected:
  void paintEvent(QPaintEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;

 private:
  RobotController* controller_{nullptr};
  QVector<QPair<QRect, int>> hit_blocks_;
};
