#pragma once

#include <QTimer>
#include <QWidget>

class TimelineWidget final : public QWidget {
 public:
  explicit TimelineWidget(QWidget* parent = nullptr);

 protected:
  void paintEvent(QPaintEvent* event) override;

 private:
  QTimer timer_;
  double playhead_{0.56};
};
