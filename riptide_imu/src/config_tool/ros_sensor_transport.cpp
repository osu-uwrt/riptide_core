#include "riptide_imu/config_tool/ros_sensor_transport.hpp"

#include <QDateTime>
#include <QJsonArray>
#include <QJsonDocument>
#include <QUuid>

namespace riptide_imu {
namespace {

QString normalizedNamespace(QString endpoint) {
  endpoint = endpoint.trimmed();
  if (endpoint.endsWith("/vectornav/config")) endpoint.chop(17);
  if (endpoint.isEmpty() || endpoint == "/") return {};
  if (!endpoint.startsWith('/')) endpoint.prepend('/');
  while (endpoint.endsWith('/')) endpoint.chop(1);
  return endpoint;
}

QStringList jsonValues(const QJsonArray& values) {
  QStringList result;
  for (const auto& value : values) result.push_back(value.toVariant().toString());
  return result;
}

}  // namespace

RosSensorTransport::RosSensorTransport(QObject* parent) : SensorInterface(parent) {}

RosSensorTransport::~RosSensorTransport() { disconnectSensor(); }

void RosSensorTransport::ensureNode() {
  if (node_) return;
  const std::string suffix = QUuid::createUuid().toString(QUuid::Id128).left(8).toStdString();
  node_ = std::make_shared<rclcpp::Node>("vn_config_ui_" + suffix);
  executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  spin_timer_ = new QTimer(this);
  spin_timer_->setInterval(5);
  connect(spin_timer_, &QTimer::timeout, this, &RosSensorTransport::spinRos);
  spin_timer_->start();
  sample_timer_ = new QTimer(this);
  sample_timer_->setInterval(200);
  connect(sample_timer_, &QTimer::timeout, this, &RosSensorTransport::pollCalibration);
  lease_timer_ = new QTimer(this);
  lease_timer_->setInterval(15000);
  connect(lease_timer_, &QTimer::timeout, this, &RosSensorTransport::renewMaintenance);
}

QString RosSensorTransport::serviceForEndpoint(const QString& endpoint) const {
  return normalizedNamespace(endpoint) + "/vectornav/config";
}

QString RosSensorTransport::topicForEndpoint(const QString& suffix) const {
  return normalizedNamespace(endpoint_) + suffix;
}

void RosSensorTransport::discoverEndpoints() {
  ensureNode();
  QStringList endpoints;
  for (const auto& service : node_->get_service_names_and_types()) {
    if (QString::fromStdString(service.first).endsWith("/vectornav/config")) {
      const QString discovered = normalizedNamespace(QString::fromStdString(service.first));
      endpoints.push_back(discovered.isEmpty() ? "/" : discovered);
    }
  }
  endpoints.removeDuplicates();
  endpoints.sort();
  emit endpointsDiscovered(endpoints);
}

void RosSensorTransport::connectSensor(const QString& endpoint, int requested_baud) {
  disconnectSensor();
  ensureNode();
  endpoint_ = normalizedNamespace(endpoint);
  requested_baud_ = requested_baud;
  client_ = node_->create_client<Query>(serviceForEndpoint(endpoint_).toStdString());
  imu_subscription_ = node_->create_subscription<sensor_msgs::msg::Imu>(
      topicForEndpoint("/vectornav/imu").toStdString(), rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr message) {
        Telemetry sample;
        sample.time_s = QDateTime::currentMSecsSinceEpoch() / 1000.0;
        sample.qx = message->orientation.x; sample.qy = message->orientation.y;
        sample.qz = message->orientation.z; sample.qw = message->orientation.w;
        sample.gx = message->angular_velocity.x; sample.gy = message->angular_velocity.y;
        sample.gz = message->angular_velocity.z;
        sample.ax = message->linear_acceleration.x; sample.ay = message->linear_acceleration.y;
        sample.az = message->linear_acceleration.z;
        emit telemetryReceived(sample);
      });
  if (!client_->service_is_ready()) {
    emit connectionChanged(false, QString("Waiting for %1").arg(serviceForEndpoint(endpoint_)));
    return;
  }
  status_pending_ = true;
  sendRequest(envelope("status"), "Connect", [this](const QJsonObject& response) {
    status_pending_ = false;
    connected_ = response.value("ok").toBool();
    if (connected_) {
      sample_timer_->start();
      const auto data = response.value("data").toObject();
      emit connectionChanged(true, QString("ROS driver %1 · sensor baud %2")
          .arg(endpoint_.isEmpty() ? "/" : endpoint_)
          .arg(data.value("baud").toInt(requested_baud_)));
    }
  });
}

void RosSensorTransport::disconnectSensor() {
  if (!session_.isEmpty() && client_) closeMaintenance();
  if (sample_timer_) sample_timer_->stop();
  if (lease_timer_) lease_timer_->stop();
  imu_subscription_.reset();
  client_.reset();
  connected_ = false;
  status_pending_ = false;
  sample_pending_ = false;
  emit connectionChanged(false, "Disconnected");
}

void RosSensorTransport::spinRos() {
  if (executor_) executor_->spin_some(std::chrono::milliseconds(1));
  if (!connected_ && !status_pending_ && client_ && client_->service_is_ready()) {
    status_pending_ = true;
    sendRequest(envelope("status"), "Connect", [this](const QJsonObject& response) {
      status_pending_ = false;
      if (!response.value("ok").toBool()) return;
      connected_ = true;
      sample_timer_->start();
      emit connectionChanged(true, QString("ROS driver %1 connected").arg(endpoint_));
    });
  }
}

QJsonObject RosSensorTransport::envelope(const QString& operation,
                                         const QJsonObject& payload,
                                         bool identified) const {
  QJsonObject request{{"version", 1}, {"operation", operation}, {"payload", payload}};
  if (!session_.isEmpty()) request["session"] = session_;
  if (identified) request["operation_id"] = QUuid::createUuid().toString(QUuid::WithoutBraces);
  return request;
}

void RosSensorTransport::sendRequest(const QJsonObject& request, const QString& operation,
                                     Reply reply) {
  if (!client_ || !client_->service_is_ready()) {
    emit operationFailed(operation, "ROS configuration service is unavailable");
    return;
  }
  auto message = std::make_shared<Query::Request>();
  message->request = QJsonDocument(request).toJson(QJsonDocument::Compact).toStdString();
  emit traffic("ROS TX", QString::fromStdString(message->request));
  auto completed = std::make_shared<bool>(false);
  const int timeout_ms = operation == "Persistent save" || operation == "Reset" ? 12000 : 3000;
  QTimer::singleShot(timeout_ms, this, [this, operation, reply, completed] {
    if (*completed) return;
    *completed = true;
    emit operationFailed(operation, "ROS configuration request timed out");
    reply(QJsonObject{{"version", 1}, {"ok", false}, {"code", "timeout"},
                      {"message", "ROS configuration request timed out"}});
  });
  client_->async_send_request(message, [this, operation, reply, completed](rclcpp::Client<Query>::SharedFuture future) {
    if (*completed) return;
    *completed = true;
    const QString text = QString::fromStdString(future.get()->response);
    emit traffic("ROS RX", text);
    QJsonParseError error;
    const QJsonObject response = QJsonDocument::fromJson(text.toUtf8(), &error).object();
    if (error.error != QJsonParseError::NoError || response.isEmpty()) {
      emit operationFailed(operation, "Driver returned an invalid response");
      return;
    }
    if (!response.value("ok").toBool()) {
      emit operationFailed(operation, response.value("message").toString());
    }
    reply(response);
  });
}

void RosSensorTransport::readRegister(int id) {
  sendRequest(envelope("read", {{"register", id}}), "Read register",
              [this, id](const QJsonObject& response) {
    if (!response.value("ok").toBool()) return;
    emit registerRead(id, jsonValues(response.value("data").toObject()
                                         .value("values").toArray()).join(';'));
  });
}

void RosSensorTransport::writeRegister(int id, const QString& semicolon_values) {
  QJsonArray values;
  for (const auto& value : semicolon_values.split(';')) values.push_back(value);
  sendRequest(envelope("write", {{"register", id}, {"values", values}}, true),
              "Write register", [this, id](const QJsonObject& response) {
    if (!response.value("ok").toBool()) return;
    emit writeVerified(id, jsonValues(response.value("data").toObject()
                                          .value("values").toArray()).join(';'));
  });
}

void RosSensorTransport::pollCalibration() {
  if (!connected_ || sample_pending_) return;
  sample_pending_ = true;
  sendRequest(envelope("read", {{"register", 54}}), "Calibration sample",
              [this](const QJsonObject& response) {
    sample_pending_ = false;
    if (!response.value("ok").toBool()) return;
    const QStringList values = jsonValues(response.value("data").toObject()
                                              .value("values").toArray());
    if (values.size() < 11) {
      emit operationFailed("Calibration sample", "Register 54 returned fewer than 11 fields");
      return;
    }
    bool ok = true;
    double parsed[11];
    for (int index = 0; index < 11; ++index) {
      bool field_ok = false;
      parsed[index] = values[index].toDouble(&field_ok);
      ok = ok && field_ok;
    }
    if (!ok) {
      emit operationFailed("Calibration sample", "Register 54 contains a non-numeric field");
      return;
    }
    Telemetry sample;
    sample.time_s = QDateTime::currentMSecsSinceEpoch() / 1000.0;
    sample.mx = parsed[0]; sample.my = parsed[1]; sample.mz = parsed[2];
    sample.ax = parsed[3]; sample.ay = parsed[4]; sample.az = parsed[5];
    sample.gx = parsed[6]; sample.gy = parsed[7]; sample.gz = parsed[8];
    sample.temperature_c = parsed[9]; sample.pressure_kpa = parsed[10] / 100.0;
    emit calibrationSampleReceived(sample);
  });
}

void RosSensorTransport::setCalibrationCapture(bool enabled) {
  if (sample_timer_) sample_timer_->setInterval(enabled ? 50 : 200);
}

void RosSensorTransport::openMaintenance() {
  sendRequest(envelope("open", {{"client_id", node_ ? QString::fromStdString(node_->get_name()) : "ui"}}),
              "Enter maintenance", [this](const QJsonObject& response) {
    if (!response.value("ok").toBool()) return;
    session_ = response.value("data").toObject().value("session").toString();
    lease_timer_->start();
    emit maintenanceChanged(true, "Maintenance session active");
  });
}

void RosSensorTransport::renewMaintenance() {
  if (session_.isEmpty()) return;
  sendRequest(envelope("renew"), "Renew maintenance", [this](const QJsonObject& response) {
    if (!response.value("ok").toBool()) {
      session_.clear(); lease_timer_->stop();
      emit maintenanceChanged(false, "Maintenance lease expired");
    }
  });
}

void RosSensorTransport::closeMaintenance() {
  if (session_.isEmpty()) return;
  sendRequest(envelope("close"), "Leave maintenance", [this](const QJsonObject&) {
    session_.clear();
    if (lease_timer_) lease_timer_->stop();
    emit maintenanceChanged(false, "Maintenance session closed");
  });
}

void RosSensorTransport::writeSettings(const QString& expected_json) {
  const auto expected = QJsonDocument::fromJson(expected_json.toUtf8()).object();
  sendRequest(envelope("save_reset", {{"expected", expected}}, true), "Persistent save",
              [this](const QJsonObject& response) {
    if (response.value("ok").toBool())
      emit operationSucceeded("Persistent save", response.value("message").toString());
  });
}

void RosSensorTransport::tare() {
  sendRequest(envelope("tare", {}, true), "Tare", [this](const QJsonObject& response) {
    if (response.value("ok").toBool()) emit operationSucceeded("Tare", "Tare complete");
  });
}

void RosSensorTransport::reset() {
  sendRequest(envelope("reset", {}, true), "Reset", [this](const QJsonObject& response) {
    if (response.value("ok").toBool()) emit operationSucceeded("Reset", "Sensor reset and reconnected");
  });
}

SensorTransportRouter::SensorTransportRouter(QObject* parent) : SensorInterface(parent) {
  serial_ = new SensorTransport(this);
  ros_ = new RosSensorTransport(this);
  connectBackend(serial_);
  connectBackend(ros_);
}

SensorInterface* SensorTransportRouter::active() const {
  return mode_ == "ros" ? static_cast<SensorInterface*>(ros_) : serial_;
}

void SensorTransportRouter::connectBackend(SensorInterface* backend) {
  connect(backend, &SensorInterface::connectionChanged, this,
          [this, backend](bool ok, const QString& detail) { if (backend == active()) emit connectionChanged(ok, detail); });
  connect(backend, &SensorInterface::telemetryReceived, this,
          [this, backend](const Telemetry& value) { if (backend == active()) emit telemetryReceived(value); });
  connect(backend, &SensorInterface::calibrationSampleReceived, this,
          [this, backend](const Telemetry& value) { if (backend == active()) emit calibrationSampleReceived(value); });
  connect(backend, &SensorInterface::registerRead, this,
          [this, backend](int id, const QString& value) { if (backend == active()) emit registerRead(id, value); });
  connect(backend, &SensorInterface::writeVerified, this,
          [this, backend](int id, const QString& value) { if (backend == active()) emit writeVerified(id, value); });
  connect(backend, &SensorInterface::operationFailed, this,
          [this, backend](const QString& op, const QString& detail) { if (backend == active()) emit operationFailed(op, detail); });
  connect(backend, &SensorInterface::traffic, this,
          [this, backend](const QString& direction, const QString& packet) { if (backend == active()) emit traffic(direction, packet); });
  connect(backend, &SensorInterface::maintenanceChanged, this,
          [this, backend](bool active_state, const QString& detail) { if (backend == active()) emit maintenanceChanged(active_state, detail); });
  connect(backend, &SensorInterface::operationSucceeded, this,
          [this, backend](const QString& op, const QString& detail) { if (backend == active()) emit operationSucceeded(op, detail); });
  connect(backend, &SensorInterface::endpointsDiscovered, this,
          [this, backend](const QStringList& endpoints) { if (backend == active()) emit endpointsDiscovered(endpoints); });
}

void SensorTransportRouter::setConnectionMode(const QString& mode) {
  active()->disconnectSensor();
  mode_ = mode == "ros" ? "ros" : "serial";
}
void SensorTransportRouter::connectSensor(const QString& target, int baud) { active()->connectSensor(target, baud); }
void SensorTransportRouter::disconnectSensor() { active()->disconnectSensor(); }
void SensorTransportRouter::discoverEndpoints() { ros_->discoverEndpoints(); }
void SensorTransportRouter::setDemoMode(bool enabled) { serial_->setDemoMode(enabled); if (enabled) mode_ = "serial"; }
void SensorTransportRouter::setCalibrationCapture(bool enabled) { active()->setCalibrationCapture(enabled); }
void SensorTransportRouter::openMaintenance() { active()->openMaintenance(); }
void SensorTransportRouter::closeMaintenance() { active()->closeMaintenance(); }
void SensorTransportRouter::readRegister(int id) { active()->readRegister(id); }
void SensorTransportRouter::writeRegister(int id, const QString& values) { active()->writeRegister(id, values); }
void SensorTransportRouter::writeSettings(const QString& expected) { active()->writeSettings(expected); }
void SensorTransportRouter::tare() { active()->tare(); }
void SensorTransportRouter::reset() { active()->reset(); }

}  // namespace riptide_imu
