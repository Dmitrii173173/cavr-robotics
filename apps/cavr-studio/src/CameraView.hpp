#pragma once

#include <QWidget>

class CameraView final : public QWidget {
 public:
  explicit CameraView(QWidget* parent = nullptr);

 protected:
  void paintEvent(QPaintEvent* event) override;
};
