#include "riptide_imu/config_tool/dashboard_widgets.hpp"

#include <QApplication>
#include <QLabel>
#include <QPainter>
#include <QPainterPath>
#include <QPalette>
#include <QStyle>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>

namespace riptide_imu {

void applyDarkTheme(QApplication* application) {
  QPalette palette;
  palette.setColor(QPalette::Window, QColor("#151a21"));
  palette.setColor(QPalette::WindowText, QColor("#e7edf5"));
  palette.setColor(QPalette::Base, QColor("#0f141a"));
  palette.setColor(QPalette::AlternateBase, QColor("#1b222c"));
  palette.setColor(QPalette::ToolTipBase, QColor("#252f3c"));
  palette.setColor(QPalette::ToolTipText, QColor("#f3f7fb"));
  palette.setColor(QPalette::Text, QColor("#dce5ef"));
  palette.setColor(QPalette::Button, QColor("#222b36"));
  palette.setColor(QPalette::ButtonText, QColor("#e7edf5"));
  palette.setColor(QPalette::BrightText, QColor("#ff6b6b"));
  palette.setColor(QPalette::Highlight, QColor("#2f81d4"));
  palette.setColor(QPalette::HighlightedText, Qt::white);
  palette.setColor(QPalette::Disabled, QPalette::Text, QColor("#697586"));
  palette.setColor(QPalette::Disabled, QPalette::ButtonText, QColor("#697586"));
  application->setPalette(palette);
  application->setStyleSheet(
      "QMainWindow,QWidget{font-size:9.5pt} QMainWindow{background:#151a21}"
      "QMenuBar,QMenu,QToolBar{background:#1b222c;color:#dce5ef;border-color:#303b49}"
      "QToolBar{spacing:4px;padding:4px;border-bottom:1px solid #303b49}"
      "QDockWidget::title{background:#202833;padding:6px;font-weight:600;border-bottom:1px solid #303b49}"
      "QTreeWidget,QTableWidget,QPlainTextEdit,QLineEdit,QComboBox{background:#0f141a;color:#dce5ef;"
      "border:1px solid #303b49;selection-background-color:#2f81d4;alternate-background-color:#171e27}"
      "QHeaderView::section{background:#202833;color:#cbd6e2;border:0;border-right:1px solid #303b49;padding:5px}"
      "QPushButton,QToolButton{background:#27313e;color:#e7edf5;border:1px solid #3a4656;border-radius:3px;padding:5px 9px}"
      "QPushButton:hover,QToolButton:hover{background:#344255;border-color:#4f6680}"
      "QPushButton:pressed,QToolButton:pressed{background:#1d6bad}"
      "QPushButton#primaryButton{background:#2f81d4;border-color:#4597e8;color:white;font-weight:600}"
      "QWidget#connectionBar{background:#1b222c;border:1px solid #303b49;border-radius:4px}"
      "QLabel#sectionTitle{font-size:11pt;font-weight:600;color:#f1f5f9}"
      "QLabel#statusGood{background:#173c31;color:#6ee7b7;border-radius:3px;padding:3px 7px}"
      "QLabel#statusStale{background:#49331a;color:#fbbf24;border-radius:3px;padding:3px 7px}"
      "QLabel#stagedBadge{background:#173b58;color:#7dd3fc;border-radius:3px;padding:2px 6px}"
      "QStatusBar{background:#10151b;color:#aebccc;border-top:1px solid #303b49}"
      "QSplitter::handle{background:#303b49;width:2px;height:2px}");
  application->setProperty("vnDarkTheme", true);
}

AttitudeWidget::AttitudeWidget(QWidget* parent) : QWidget(parent) {
  setMinimumSize(220, 160);
}

void AttitudeWidget::setQuaternion(double x, double y, double z, double w) {
  Eigen::Quaterniond q(w, x, y, z);
  if (q.norm() > 1e-9 && q.coeffs().allFinite()) orientation_ = q.normalized();
  update();
}

void AttitudeWidget::paintEvent(QPaintEvent*) {
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.fillRect(rect(), palette().base());
  painter.translate(width() / 2.0, height() / 2.0);
  const Eigen::Matrix3d rotation = orientation_.toRotationMatrix();
  auto project = [&](const Eigen::Vector3d& value) {
    const Eigen::Vector3d rotated = rotation * value;
    return QPointF(68.0 * (rotated.x() + 0.35 * rotated.z()),
                   -68.0 * (rotated.y() - 0.2 * rotated.z()));
  };
  const Eigen::Vector3d vertices[] = {
      {-1, -0.55, -0.25}, {1, -0.55, -0.25}, {1, 0.55, -0.25}, {-1, 0.55, -0.25},
      {-1, -0.55, 0.25}, {1, -0.55, 0.25}, {1, 0.55, 0.25}, {-1, 0.55, 0.25}};
  const int edges[][2] = {{0,1},{1,2},{2,3},{3,0},{4,5},{5,6},{6,7},{7,4},{0,4},{1,5},{2,6},{3,7}};
  painter.setPen(QPen(QColor("#7f95ad"), 3));
  for (const auto& edge : edges) painter.drawLine(project(vertices[edge[0]]), project(vertices[edge[1]]));
  const QColor colors[] = {QColor("#ff6b6b"), QColor("#5ee0a0"), QColor("#62b5f5")};
  for (int axis = 0; axis < 3; ++axis) {
    painter.setPen(QPen(colors[axis], 4));
    painter.drawLine(QPointF(0, 0), project(Eigen::Vector3d::Unit(axis) * 1.4));
  }
  painter.setPen(palette().text().color());
  painter.drawText(QRectF(-width()/2.0 + 10, -height()/2.0 + 6, width()-20, 24),
                   "Sensor attitude · position not inferred");
}

PlotWidget::PlotWidget(QWidget* parent) : QWidget(parent) { setMinimumHeight(140); }

void PlotWidget::append(const Telemetry& sample) {
  samples_.push_back(sample);
  while (samples_.size() > 300) samples_.pop_front();
  update();
}

void PlotWidget::paintEvent(QPaintEvent*) {
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.fillRect(rect(), palette().base());
  const QRectF area = rect().adjusted(48, 22, -15, -28);
  painter.setPen(QPen(QColor("#2e3947"), 1));
  for (int i = 0; i <= 4; ++i)
    painter.drawLine(area.left(), area.top() + i * area.height()/4.0,
                     area.right(), area.top() + i * area.height()/4.0);
  if (samples_.size() < 2) return;
  double maximum = 0.1;
  for (const auto& sample : samples_)
    maximum = std::max({maximum, std::abs(sample.gx), std::abs(sample.gy), std::abs(sample.gz)});
  const QColor colors[] = {QColor("#ff6b6b"), QColor("#5ee0a0"), QColor("#62b5f5")};
  for (int axis = 0; axis < 3; ++axis) {
    QPainterPath path;
    for (std::size_t i = 0; i < samples_.size(); ++i) {
      const auto& sample = samples_[i];
      const double value = axis == 0 ? sample.gx : axis == 1 ? sample.gy : sample.gz;
      const QPointF point(area.left() + i * area.width()/(samples_.size()-1),
                          area.center().y() - value * area.height()/(2.0*maximum));
      i == 0 ? path.moveTo(point) : path.lineTo(point);
    }
    painter.setPen(QPen(colors[axis], 2));
    painter.drawPath(path);
  }
  painter.setPen(palette().text().color());
  painter.drawText(8, 18, "Angular rate (rad/s)");
}

TelemetrySummaryWidget::TelemetrySummaryWidget(QWidget* parent) : QWidget(parent) {
  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(8, 8, 8, 8);
  connection_badge_ = new QLabel("WAITING FOR DATA");
  connection_badge_->setObjectName("statusStale");
  connection_badge_->setAlignment(Qt::AlignCenter);
  values_ = new QLabel("No telemetry received");
  values_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  statistics_ = new QLabel("Rate — Hz  ·  Samples 0");
  statistics_->setStyleSheet("color:#8fa0b4");
  layout->addWidget(connection_badge_);
  layout->addWidget(values_, 1);
  layout->addWidget(statistics_);
  rate_clock_.start();
}

void TelemetrySummaryWidget::updateSample(const Telemetry& sample) {
  ++samples_in_window_;
  if (rate_clock_.elapsed() >= 1000) {
    sample_rate_hz_ = 1000.0 * samples_in_window_ / rate_clock_.elapsed();
    samples_in_window_ = 0;
    rate_clock_.restart();
  }
  connection_badge_->setText("LIVE");
  connection_badge_->setObjectName("statusGood");
  connection_badge_->style()->unpolish(connection_badge_);
  connection_badge_->style()->polish(connection_badge_);
  values_->setText(QString(
      "ACCELERATION  m/s²\n  X  %1\n  Y  %2\n  Z  %3\n\n"
      "ANGULAR RATE  rad/s\n  X  %4\n  Y  %5\n  Z  %6\n\n"
      "MAGNETIC  G\n  X  %7\n  Y  %8\n  Z  %9\n\n"
      "TEMP  %10 °C    PRESS  %11 kPa")
      .arg(sample.ax,0,'f',4).arg(sample.ay,0,'f',4).arg(sample.az,0,'f',4)
      .arg(sample.gx,0,'f',6).arg(sample.gy,0,'f',6).arg(sample.gz,0,'f',6)
      .arg(sample.mx,0,'f',4).arg(sample.my,0,'f',4).arg(sample.mz,0,'f',4)
      .arg(sample.temperature_c,0,'f',2).arg(sample.pressure_kpa,0,'f',3));
  statistics_->setText(QString("Rate %1 Hz  ·  Window %2 samples")
                           .arg(sample_rate_hz_, 0, 'f', 1).arg(samples_in_window_));
}

void TelemetrySummaryWidget::markStale(bool stale) {
  if (!stale) return;
  connection_badge_->setText("STALE");
  connection_badge_->setObjectName("statusStale");
  connection_badge_->style()->unpolish(connection_badge_);
  connection_badge_->style()->polish(connection_badge_);
}

}  // namespace riptide_imu
