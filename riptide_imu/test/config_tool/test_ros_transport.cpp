#include <QtTest/QtTest>

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSignalSpy>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>
#include <riptide_msgs2/srv/query_imu_serial.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "riptide_imu/config_tool/ros_sensor_transport.hpp"

namespace riptide_imu {

class RosTransportTest : public QObject {
  Q_OBJECT
 private slots:
  void initTestCase() {
    if (!rclcpp::ok()) {
      int argc = 1;
      char name[] = "test_ros_transport";
      char* argv[] = {name, nullptr};
      rclcpp::init(argc, argv);
    }
    qRegisterMetaType<Telemetry>();
  }

  void cleanupTestCase() {
    if (rclcpp::ok()) rclcpp::shutdown();
  }

  void discoversConnectsAndKeepsCalibrationChannelSeparate() {
    using Query = riptide_msgs2::srv::QueryImuSerial;
    auto server_node = std::make_shared<rclcpp::Node>("fake_vn_driver", "/robot");
    bool maintenance = false;
    auto service = server_node->create_service<Query>(
        "vectornav/config",
        [&maintenance](const std::shared_ptr<Query::Request> request,
                       std::shared_ptr<Query::Response> response) {
          const auto document = QJsonDocument::fromJson(
              QByteArray::fromStdString(request->request)).object();
          const QString operation = document.value("operation").toString();
          QJsonObject data;
          bool ok = true;
          QString message = "ok";
          if (operation == "status") {
            data = {{"baud", 460800}, {"publishing", true}};
          } else if (operation == "open") {
            maintenance = true;
            data = {{"session", "test-session"}, {"lease_ms", 30000}};
          } else if (operation == "close" || operation == "renew") {
            maintenance = operation == "renew";
          } else if (operation == "read") {
            const int id = document.value("payload").toObject().value("register").toInt();
            QJsonArray values;
            if (id == 54) {
              for (const auto* value : {"1", "2", "3", "4", "5", "6",
                                        "0.01", "0.02", "0.03", "24", "101300"})
                values.push_back(value);
            } else {
              values.push_back("value");
            }
            data = {{"register", id}, {"values", values}};
          } else if (operation == "write") {
            const auto payload = document.value("payload").toObject();
            const int id = payload.value("register").toInt();
            if (id == 5 && !maintenance) {
              ok = false;
              message = "driver-critical register requires maintenance mode";
            } else {
              data = {{"register", id}, {"values", payload.value("values")}};
            }
          } else if (operation == "save_reset") {
            if (!maintenance) { ok = false; message = "maintenance required"; }
          }
          response->response = QString(QJsonDocument(QJsonObject{
              {"version", 1}, {"ok", ok}, {"code", ok ? "ok" : "error"},
              {"message", message}, {"data", data}}).toJson(QJsonDocument::Compact)).toStdString();
        });
    auto publisher = server_node->create_publisher<sensor_msgs::msg::Imu>(
        "vectornav/imu", rclcpp::SensorDataQoS());
    rclcpp::executors::SingleThreadedExecutor server_executor;
    server_executor.add_node(server_node);
    QTimer server_spin;
    connect(&server_spin, &QTimer::timeout, [&] { server_executor.spin_some(); });
    server_spin.start(2);

    RosSensorTransport transport;
    QSignalSpy endpoints(&transport, &SensorInterface::endpointsDiscovered);
    QSignalSpy connection(&transport, &SensorInterface::connectionChanged);
    QSignalSpy display(&transport, &SensorInterface::telemetryReceived);
    QSignalSpy calibration(&transport, &SensorInterface::calibrationSampleReceived);
    QSignalSpy failures(&transport, &SensorInterface::operationFailed);
    QSignalSpy maintenance_spy(&transport, &SensorInterface::maintenanceChanged);
    QSignalSpy writes(&transport, &SensorInterface::writeVerified);

    QTest::qWait(100);
    transport.discoverEndpoints();
    QTRY_VERIFY_WITH_TIMEOUT(!endpoints.isEmpty(), 2000);
    QVERIFY(endpoints.takeFirst().at(0).toStringList().contains("/robot"));

    transport.connectSensor("/robot", 460800);
    QTRY_VERIFY_WITH_TIMEOUT(!connection.isEmpty(), 3000);
    QTRY_VERIFY_WITH_TIMEOUT(connection.last().at(0).toBool(), 3000);

    sensor_msgs::msg::Imu message;
    message.orientation.w = 1.0;
    message.angular_velocity.x = 9.0;
    message.linear_acceleration.z = 8.0;
    publisher->publish(message);
    QTRY_VERIFY_WITH_TIMEOUT(!display.isEmpty(), 2000);
    QCOMPARE(display.last().at(0).value<Telemetry>().gx, 9.0);

    QTRY_VERIFY_WITH_TIMEOUT(!calibration.isEmpty(), 3000);
    const Telemetry calibration_sample = calibration.last().at(0).value<Telemetry>();
    QCOMPARE(calibration_sample.gx, 0.01);
    QCOMPARE(calibration_sample.ax, 4.0);
    QCOMPARE(calibration_sample.temperature_c, 24.0);

    transport.writeRegister(5, "115200;115200");
    QTRY_VERIFY_WITH_TIMEOUT(!failures.isEmpty(), 2000);
    transport.openMaintenance();
    QTRY_VERIFY_WITH_TIMEOUT(!maintenance_spy.isEmpty(), 2000);
    transport.writeRegister(5, "115200;115200");
    QTRY_VERIFY_WITH_TIMEOUT(!writes.isEmpty(), 2000);
    QCOMPARE(writes.last().at(0).toInt(), 5);
  }
};

}  // namespace riptide_imu

QTEST_MAIN(riptide_imu::RosTransportTest)
#include "test_ros_transport.moc"
