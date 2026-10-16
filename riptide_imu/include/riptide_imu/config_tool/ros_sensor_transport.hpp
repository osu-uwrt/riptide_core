#pragma once

#include <functional>
#include <memory>

#include <QJsonObject>
#include <QPointer>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>
#include <riptide_msgs2/srv/query_imu_serial.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "riptide_imu/config_tool/sensor_transport.hpp"

namespace riptide_imu {

class RosSensorTransport : public SensorInterface {
  Q_OBJECT
 public:
  explicit RosSensorTransport(QObject* parent = nullptr);
  ~RosSensorTransport() override;

 public slots:
  void connectSensor(const QString& endpoint, int requested_baud) override;
  void disconnectSensor() override;
  void setConnectionMode(const QString&) override {}
  void discoverEndpoints() override;
  void setDemoMode(bool) override {}
  void setCalibrationCapture(bool enabled) override;
  void openMaintenance() override;
  void closeMaintenance() override;
  void readRegister(int id) override;
  void writeRegister(int id, const QString& semicolon_values) override;
  void writeSettings(const QString& expected_json) override;
  void tare() override;
  void reset() override;

 private slots:
  void spinRos();
  void pollCalibration();
  void renewMaintenance();

 private:
  using Query = riptide_msgs2::srv::QueryImuSerial;
  using Reply = std::function<void(const QJsonObject&)>;
  void ensureNode();
  void sendRequest(const QJsonObject& request, const QString& operation, Reply reply);
  QJsonObject envelope(const QString& operation, const QJsonObject& payload = {},
                       bool identified = false) const;
  QString serviceForEndpoint(const QString& endpoint) const;
  QString topicForEndpoint(const QString& suffix) const;
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Client<Query>::SharedPtr client_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  QTimer* spin_timer_{nullptr};
  QTimer* sample_timer_{nullptr};
  QTimer* lease_timer_{nullptr};
  QString endpoint_;
  QString session_;
  bool connected_{false};
  bool status_pending_{false};
  bool sample_pending_{false};
  int requested_baud_{0};
};

class SensorTransportRouter : public SensorInterface {
  Q_OBJECT
 public:
  explicit SensorTransportRouter(QObject* parent = nullptr);

 public slots:
  void connectSensor(const QString& target, int baud) override;
  void disconnectSensor() override;
  void setConnectionMode(const QString& mode) override;
  void discoverEndpoints() override;
  void setDemoMode(bool enabled) override;
  void setCalibrationCapture(bool enabled) override;
  void openMaintenance() override;
  void closeMaintenance() override;
  void readRegister(int id) override;
  void writeRegister(int id, const QString& values) override;
  void writeSettings(const QString& expected_json) override;
  void tare() override;
  void reset() override;

 private:
  SensorInterface* active() const;
  void connectBackend(SensorInterface* backend);
  SensorTransport* serial_{nullptr};
  RosSensorTransport* ros_{nullptr};
  QString mode_{"ros"};
};

}  // namespace riptide_imu
