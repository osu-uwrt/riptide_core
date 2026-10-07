#include "riptide_imu/config_tool/sensor_transport.hpp"

#include <QDateTime>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace riptide_imu {

SensorTransport::SensorTransport(QObject* parent) : QObject(parent) {}

SensorTransport::~SensorTransport() { disconnectSensor(); }

void SensorTransport::connectSensor(const QString& port, int requested_baud) {
  disconnectSensor();
  if (!poll_timer_) {
    poll_timer_ = new QTimer(this);
    poll_timer_->setInterval(50);
    connect(poll_timer_, &QTimer::timeout, this, &SensorTransport::poll);
  }
  if (demo_) {
    poll_timer_->start();
    emit connectionChanged(true, "Demo data (sensor writes disabled)");
    return;
  }
  try {
    sensor_ = std::make_unique<vn::sensors::VnSensor>();
    auto rates = sensor_->supportedBaudrates();
    rates.erase(std::remove(rates.begin(), rates.end(), requested_baud), rates.end());
    rates.insert(rates.begin(), requested_baud);
    QString failures;
    for (const auto rate : rates) {
      try {
        sensor_->connect(port.toStdString(), rate);
        if (sensor_->verifySensorConnectivity()) {
          const QString identity = QString::fromStdString(sensor_->readModelNumber());
          poll_timer_->start();
          emit connectionChanged(true, QString("%1 at %2 baud").arg(identity).arg(rate));
          return;
        }
        sensor_->disconnect();
      } catch (const std::exception& exception) {
        failures += QString("%1: %2; ").arg(rate).arg(exception.what());
        try { if (sensor_->isConnected()) sensor_->disconnect(); } catch (...) {}
      }
    }
    sensor_.reset();
    emit operationFailed("Connect", failures.isEmpty() ? "No supported baud rate responded" : failures);
  } catch (const std::exception& exception) {
    sensor_.reset();
    emit operationFailed("Connect", exception.what());
  }
}

void SensorTransport::disconnectSensor() {
  if (poll_timer_) poll_timer_->stop();
  try { if (sensor_ && sensor_->isConnected()) sensor_->disconnect(); } catch (...) {}
  sensor_.reset();
  emit connectionChanged(false, "Disconnected");
}

void SensorTransport::setDemoMode(bool enabled) { demo_ = enabled; }

QString SensorTransport::transact(const QString& command) {
  if (!sensor_) throw std::runtime_error("Sensor is not connected");
  emit traffic("TX", command);
  const QString response = QString::fromStdString(sensor_->transaction(command.toStdString()));
  emit traffic("RX", response.trimmed());
  return response;
}

QString SensorTransport::valuesFromResponse(const QString& response, int id) {
  QString text = response.trimmed();
  const int checksum = text.indexOf('*');
  if (checksum >= 0) text.truncate(checksum);
  if (text.startsWith('$')) text.remove(0, 1);
  const auto fields = text.split(',');
  if (fields.size() < 3 || fields[0] != "VNRRG" || fields[1].toInt() != id)
    throw std::runtime_error("Unexpected register response");
  return fields.mid(2).join(';');
}

void SensorTransport::readRegister(int id) {
  try { emit registerRead(id, valuesFromResponse(transact(QString("VNRRG,%1").arg(id)), id)); }
  catch (const std::exception& exception) { emit operationFailed("Read register", exception.what()); }
}

void SensorTransport::writeRegister(int id, const QString& semicolon_values) {
  if (demo_) { emit operationFailed("Write register", "Writes are disabled in demo mode"); return; }
  try {
    const QString wire_values = QString(semicolon_values).replace(';', ',');
    transact(QString("VNWRG,%1,%2").arg(id).arg(wire_values));
    const QString verified = valuesFromResponse(transact(QString("VNRRG,%1").arg(id)), id);
    if (verified != semicolon_values)
      throw std::runtime_error("Readback differs from staged value");
    emit writeVerified(id, verified);
  } catch (const std::exception& exception) { emit operationFailed("Write register", exception.what()); }
}

void SensorTransport::writeSettings() {
  if (demo_) { emit operationFailed("Save settings", "Writes are disabled in demo mode"); return; }
  try { transact("VNWNV"); }
  catch (const std::exception& exception) { emit operationFailed("Save settings", exception.what()); }
}

void SensorTransport::tare() {
  if (demo_) return;
  try { transact("VNTAR"); }
  catch (const std::exception& exception) { emit operationFailed("Tare", exception.what()); }
}

void SensorTransport::reset() {
  if (demo_) return;
  try { transact("VNRST"); }
  catch (const std::exception& exception) { emit operationFailed("Reset", exception.what()); }
}

void SensorTransport::poll() {
  Telemetry output;
  output.time_s = QDateTime::currentMSecsSinceEpoch() / 1000.0;
  if (demo_) {
    demo_time_ += 0.05;
    const double roll = 0.35 * std::sin(demo_time_ * 0.7);
    const double pitch = 0.22 * std::sin(demo_time_ * 0.43);
    output.qx = std::sin(roll / 2.0) * std::cos(pitch / 2.0);
    output.qy = std::cos(roll / 2.0) * std::sin(pitch / 2.0);
    output.qz = -std::sin(roll / 2.0) * std::sin(pitch / 2.0);
    output.qw = std::cos(roll / 2.0) * std::cos(pitch / 2.0);
    output.ax = 9.81 * std::sin(pitch) + 0.01 * std::sin(demo_time_ * 9.0);
    output.ay = -9.81 * std::sin(roll) * std::cos(pitch);
    output.az = 9.81 * std::cos(roll) * std::cos(pitch);
    output.gx = 0.245 * std::cos(demo_time_ * 0.7);
    output.gy = 0.095 * std::cos(demo_time_ * 0.43);
    output.mx = 0.3 * std::cos(demo_time_); output.my = 0.3 * std::sin(demo_time_); output.mz = 0.4;
    output.temperature_c = 24.5; output.pressure_kpa = 101.3;
    emit telemetryReceived(output);
    return;
  }
  if (!sensor_) return;
  try {
    const auto q = sensor_->readAttitudeQuaternion();
    const auto imu = sensor_->readImuMeasurements();
    output.qx = q[0]; output.qy = q[1]; output.qz = q[2]; output.qw = q[3];
    output.mx = imu.mag[0]; output.my = imu.mag[1]; output.mz = imu.mag[2];
    output.ax = imu.accel[0]; output.ay = imu.accel[1]; output.az = imu.accel[2];
    output.gx = imu.gyro[0]; output.gy = imu.gyro[1]; output.gz = imu.gyro[2];
    output.temperature_c = imu.temp; output.pressure_kpa = imu.pressure;
    emit telemetryReceived(output);
  } catch (const std::exception& exception) {
    poll_timer_->stop();
    emit connectionChanged(false, "Connection lost; reconnect explicitly after checking port ownership");
    emit operationFailed("Telemetry", exception.what());
  }
}

}  // namespace riptide_imu
