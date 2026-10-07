#pragma once

#include <QElapsedTimer>
#include <QWidget>
#include <Eigen/Geometry>
#include <deque>

#include "riptide_imu/config_tool/sensor_transport.hpp"

class QApplication;
class QLabel;

namespace riptide_imu {

void applyDarkTheme(QApplication* application);

class AttitudeWidget : public QWidget {
  Q_OBJECT
 public:
  explicit AttitudeWidget(QWidget* parent = nullptr);
  void setQuaternion(double x, double y, double z, double w);

 protected:
  void paintEvent(QPaintEvent*) override;

 private:
  Eigen::Quaterniond orientation_{Eigen::Quaterniond::Identity()};
};

class PlotWidget : public QWidget {
  Q_OBJECT
 public:
  explicit PlotWidget(QWidget* parent = nullptr);
  void append(const Telemetry& sample);

 protected:
  void paintEvent(QPaintEvent*) override;

 private:
  std::deque<Telemetry> samples_;
};

class TelemetrySummaryWidget : public QWidget {
  Q_OBJECT
 public:
  explicit TelemetrySummaryWidget(QWidget* parent = nullptr);
  void updateSample(const Telemetry& sample);
  void markStale(bool stale);

 private:
  QLabel* connection_badge_{nullptr};
  QLabel* values_{nullptr};
  QLabel* statistics_{nullptr};
  QElapsedTimer rate_clock_;
  int samples_in_window_{0};
  double sample_rate_hz_{0.0};
};

}  // namespace riptide_imu
