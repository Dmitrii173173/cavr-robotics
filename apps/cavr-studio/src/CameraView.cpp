#include "CameraView.hpp"

#include <QPainter>
#include <QPaintEvent>

CameraView::CameraView(QWidget* parent) : QWidget(parent) {
  setMinimumSize(320, 220);
}

void CameraView::paintEvent(QPaintEvent* event) {
  Q_UNUSED(event);
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.fillRect(rect(), QColor("#141a20"));

  painter.save();
  painter.translate(width() * 0.5, height() * 0.52);
  painter.rotate(-10);
  painter.setPen(QPen(QColor("#666f77"), 1));
  painter.setBrush(QColor("#3b4146"));
  painter.drawRoundedRect(QRectF(-185, -72, 370, 144), 4, 4);

  painter.setPen(QPen(QColor("#c7c3b3"), 16, Qt::SolidLine, Qt::RoundCap));
  painter.drawLine(QPointF(-145, -34), QPointF(145, 24));

  painter.setPen(QPen(QColor(255, 255, 255, 56), 2));
  for (int offset = -130; offset <= 130; offset += 20) {
    painter.drawLine(QPointF(offset, -44), QPointF(offset + 30, 35));
  }

  painter.setBrush(QColor("#090b0d"));
  painter.setPen(Qt::NoPen);
  for (const QPointF& point : {QPointF(-125, 40), QPointF(-50, -43), QPointF(82, -46), QPointF(150, 38)}) {
    painter.drawEllipse(point, 25, 15);
  }
  painter.restore();

  painter.setPen(QPen(QColor(255, 77, 69, 210), 1));
  painter.drawLine(QPointF(width() * 0.5 - 42, height() * 0.5), QPointF(width() * 0.5 + 42, height() * 0.5));
  painter.drawLine(QPointF(width() * 0.5, height() * 0.5 - 42), QPointF(width() * 0.5, height() * 0.5 + 42));

  painter.setPen(QColor("#d7e1ea"));
  painter.drawText(QRect(12, 10, width() - 24, 22), Qt::AlignLeft | Qt::AlignVCenter, "RGB");
}
