#pragma once

#include <QObject>
#include <QTimer>
#include <memory>

#include <vn/sensors.h>

namespace riptide_imu {

struct Telemetry {
  double time_s{0.0};
  double qx{0.0}, qy{0.0}, qz{0.0}, qw{1.0};
  double mx{0.0}, my{0.0}, mz{0.0};
  double ax{0.0}, ay{0.0}, az{0.0};
  double gx{0.0}, gy{0.0}, gz{0.0};
  double temperature_c{0.0}, pressure_kpa{0.0};
};

class SensorInterface : public QObject {
  Q_OBJECT
 public:
  explicit SensorInterface(QObject* parent = nullptr) : QObject(parent) {}
  ~SensorInterface() override = default;

 public slots:
  virtual void connectSensor(const QString& port, int requested_baud) = 0;
  virtual void disconnectSensor() = 0;
  virtual void setDemoMode(bool enabled) = 0;
  virtual void readRegister(int id) = 0;
  virtual void writeRegister(int id, const QString& semicolon_values) = 0;
  virtual void writeSettings() = 0;
  virtual void tare() = 0;
  virtual void reset() = 0;

 signals:
  void connectionChanged(bool connected, const QString& detail);
  void telemetryReceived(const riptide_imu::Telemetry& telemetry);
  void registerRead(int id, const QString& values);
  void operationFailed(const QString& operation, const QString& detail);
  void traffic(const QString& direction, const QString& packet);
  void writeVerified(int id, const QString& values);
};

class SensorTransport : public SensorInterface {
  Q_OBJECT
 public:
  explicit SensorTransport(QObject* parent = nullptr);
  ~SensorTransport() override;

 public slots:
  void connectSensor(const QString& port, int requested_baud) override;
  void disconnectSensor() override;
  void setDemoMode(bool enabled) override;
  void readRegister(int id) override;
  void writeRegister(int id, const QString& semicolon_values) override;
  void writeSettings() override;
  void tare() override;
  void reset() override;

 private slots:
  void poll();

 private:
  QString transact(const QString& command);
  static QString valuesFromResponse(const QString& response, int id);
  std::unique_ptr<vn::sensors::VnSensor> sensor_;
  QTimer* poll_timer_{nullptr};
  bool demo_{false};
  double demo_time_{0.0};
};

}  // namespace riptide_imu

Q_DECLARE_METATYPE(riptide_imu::Telemetry)
