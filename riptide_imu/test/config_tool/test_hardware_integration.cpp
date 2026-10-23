#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSignalSpy>
#include <QtTest>
#include <rclcpp/rclcpp.hpp>

#include "riptide_imu/config_tool/configuration.hpp"
#include "riptide_imu/config_tool/ros_sensor_transport.hpp"
#include "riptide_imu/config_tool/sensor_transport.hpp"

using namespace riptide_imu;

class HardwareIntegrationTest : public QObject {
  Q_OBJECT
 private:
  QJsonObject snapshot() const {
    const QString path = qEnvironmentVariable("VN_HARDWARE_SNAPSHOT");
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) return {};
    return QJsonDocument::fromJson(file.readAll()).object().value("active_snapshot").toObject();
  }

 private slots:
  void initTestCase() {
    if (!rclcpp::ok()) {
      int argc = 1;
      char name[] = "test_hardware_integration";
      char* argv[] = {name, nullptr};
      rclcpp::init(argc, argv);
    }
    qRegisterMetaType<Telemetry>();
  }

  void cleanupTestCase() { if (rclcpp::ok()) rclcpp::shutdown(); }

  void directSerialReadAndTelemetry() {
    const QString port = qEnvironmentVariable("VN_HARDWARE_PORT");
    if (port.isEmpty()) QSKIP("Set VN_HARDWARE_PORT for physical-sensor tests");
    const auto expected = snapshot();
    QVERIFY2(!expected.isEmpty(), "VN_HARDWARE_SNAPSHOT must name the pre-test journal");
    SensorTransport transport;
    QSignalSpy connection(&transport, &SensorInterface::connectionChanged);
    QSignalSpy reads(&transport, &SensorInterface::registerRead);
    QSignalSpy failures(&transport, &SensorInterface::operationFailed);
    QSignalSpy samples(&transport, &SensorInterface::calibrationSampleReceived);
    transport.connectSensor(port, 460800);
    QVERIFY2(!connection.isEmpty() && connection.last()[0].toBool(), "Direct serial connection failed");
    for (auto entry = expected.begin(); entry != expected.end(); ++entry) {
      const int id = entry.key().toInt();
      if (id == 80) continue;
      const int before = reads.size();
      transport.readRegister(id);
      QVERIFY2(reads.size() == before + 1,
               qPrintable(QString("Register %1 read failed: %2").arg(id).arg(
                   failures.isEmpty() ? "no detail" : failures.last()[1].toString())));
      QCOMPARE(reads.last()[0].toInt(), id);
      QVERIFY2(!reads.last()[1].toString().isEmpty(), qPrintable(QString("Register %1 was empty").arg(id)));
    }
    QTRY_VERIFY_WITH_TIMEOUT(!samples.isEmpty(), 1500);
    transport.disconnectSensor();
  }

  void rosDriverReadAndTelemetry() {
    if (!qEnvironmentVariableIsSet("VN_HARDWARE_ROS"))
      QSKIP("Set VN_HARDWARE_ROS for a running physical-sensor driver");
    const auto expected = snapshot();
    QVERIFY(!expected.isEmpty());
    RosSensorTransport transport;
    QSignalSpy connection(&transport, &SensorInterface::connectionChanged);
    QSignalSpy display(&transport, &SensorInterface::telemetryReceived);
    QSignalSpy calibration(&transport, &SensorInterface::calibrationSampleReceived);
    QSignalSpy reads(&transport, &SensorInterface::registerRead);
    transport.connectSensor(qEnvironmentVariable("VN_HARDWARE_NAMESPACE"), 460800);
    QTRY_VERIFY_WITH_TIMEOUT(!connection.isEmpty() && connection.last()[0].toBool(), 5000);
    QTRY_VERIFY_WITH_TIMEOUT(!display.isEmpty(), 3000);
    QTRY_VERIFY_WITH_TIMEOUT(!calibration.isEmpty(), 3000);
    for (auto entry = expected.begin(); entry != expected.end(); ++entry) {
      const int id = entry.key().toInt();
      if (id == 80) continue;
      const int before = reads.size();
      transport.readRegister(id);
      QTRY_VERIFY_WITH_TIMEOUT(reads.size() == before + 1, 3000);
      QCOMPARE(reads.last()[0].toInt(), id);
    }
    transport.disconnectSensor();
  }

  void rosDriverSameValueWritesEveryPersistentRegister() {
    if (!qEnvironmentVariableIsSet("VN_HARDWARE_WRITES"))
      QSKIP("Set VN_HARDWARE_WRITES after taking a recovery snapshot");
    const auto expected = snapshot();
    QString schema_error;
    const auto definitions = loadRegisterDefinitions(qEnvironmentVariable("VN_HARDWARE_SCHEMA"),
                                                     &schema_error);
    QVERIFY2(!definitions.isEmpty(), qPrintable(schema_error));
    RosSensorTransport transport;
    QSignalSpy connection(&transport, &SensorInterface::connectionChanged);
    QSignalSpy maintenance(&transport, &SensorInterface::maintenanceChanged);
    QSignalSpy writes(&transport, &SensorInterface::writeVerified);
    QSignalSpy failures(&transport, &SensorInterface::operationFailed);
    transport.connectSensor(qEnvironmentVariable("VN_HARDWARE_NAMESPACE"), 460800);
    QTRY_VERIFY_WITH_TIMEOUT(!connection.isEmpty() && connection.last()[0].toBool(), 5000);
    transport.openMaintenance();
    QTRY_VERIFY_WITH_TIMEOUT(!maintenance.isEmpty() && maintenance.last()[0].toBool(), 3000);
    for (const auto& definition : definitions) {
      const QString key = QString::number(definition.id);
      if (!definition.writable || !definition.persistent || !expected.contains(key)) continue;
      QStringList values;
      for (const auto value : expected.value(key).toArray()) values.push_back(value.toString());
      const int before = writes.size();
      transport.writeRegister(definition.id, values.join(';'));
      QTRY_VERIFY_WITH_TIMEOUT(writes.size() == before + 1, 5000);
      QCOMPARE(writes.last()[0].toInt(), definition.id);
    }
    QVERIFY2(failures.isEmpty(), qPrintable(failures.isEmpty() ? QString() : failures.last()[1].toString()));
    transport.closeMaintenance();
    QTRY_VERIFY_WITH_TIMEOUT(!maintenance.isEmpty() && !maintenance.last()[0].toBool(), 3000);
    transport.disconnectSensor();
  }

  void directSerialSameValueWritesEveryPersistentRegister() {
    const QString port = qEnvironmentVariable("VN_HARDWARE_PORT");
    if (port.isEmpty() || !qEnvironmentVariableIsSet("VN_HARDWARE_DIRECT_WRITES"))
      QSKIP("Set VN_HARDWARE_PORT and VN_HARDWARE_DIRECT_WRITES after snapshotting");
    const auto expected = snapshot();
    QString schema_error;
    const auto definitions = loadRegisterDefinitions(qEnvironmentVariable("VN_HARDWARE_SCHEMA"),
                                                     &schema_error);
    QVERIFY2(!definitions.isEmpty(), qPrintable(schema_error));
    SensorTransport transport;
    QSignalSpy connection(&transport, &SensorInterface::connectionChanged);
    QSignalSpy writes(&transport, &SensorInterface::writeVerified);
    QSignalSpy failures(&transport, &SensorInterface::operationFailed);
    transport.connectSensor(port, 460800);
    QVERIFY(!connection.isEmpty() && connection.last()[0].toBool());
    for (const auto& definition : definitions) {
      const QString key = QString::number(definition.id);
      if (!definition.writable || !definition.persistent || !expected.contains(key)) continue;
      QStringList values;
      for (const auto value : expected.value(key).toArray()) values.push_back(value.toString());
      const int before = writes.size();
      transport.writeRegister(definition.id, values.join(';'));
      QVERIFY2(writes.size() == before + 1,
               qPrintable(QString("Register %1 write failed: %2").arg(definition.id).arg(
                   failures.isEmpty() ? "no detail" : failures.last()[1].toString())));
    }
    transport.disconnectSensor();
  }

  void directSerialBaudTransitionAndRestore() {
    const QString port = qEnvironmentVariable("VN_HARDWARE_PORT");
    if (port.isEmpty() || !qEnvironmentVariableIsSet("VN_HARDWARE_BAUD_TEST"))
      QSKIP("Set VN_HARDWARE_PORT and VN_HARDWARE_BAUD_TEST after snapshotting");
    SensorTransport transport;
    QSignalSpy connection(&transport, &SensorInterface::connectionChanged);
    QSignalSpy writes(&transport, &SensorInterface::writeVerified);
    transport.connectSensor(port, 460800);
    QVERIFY(!connection.isEmpty() && connection.last()[0].toBool());
    transport.writeRegister(5, "230400");
    QVERIFY(!writes.isEmpty() && writes.last()[1].toString() == "230400");
    transport.writeRegister(5, "460800");
    QVERIFY(writes.size() >= 2 && writes.last()[1].toString() == "460800");
    transport.disconnectSensor();
  }

  void directSerialPersistentSaveResetAndReadback() {
    const QString port = qEnvironmentVariable("VN_HARDWARE_PORT");
    if (port.isEmpty() || !qEnvironmentVariableIsSet("VN_HARDWARE_DIRECT_PERSISTENCE"))
      QSKIP("Set VN_HARDWARE_PORT and VN_HARDWARE_DIRECT_PERSISTENCE after snapshotting");
    const auto active = snapshot();
    QString schema_error;
    const auto definitions = loadRegisterDefinitions(qEnvironmentVariable("VN_HARDWARE_SCHEMA"),
                                                     &schema_error);
    QVERIFY2(!definitions.isEmpty(), qPrintable(schema_error));
    QJsonObject expected;
    for (const auto& definition : definitions) {
      const QString key = QString::number(definition.id);
      if (definition.writable && definition.persistent && active.contains(key))
        expected.insert(key, active.value(key));
    }
    SensorTransport transport;
    QSignalSpy connection(&transport, &SensorInterface::connectionChanged);
    QSignalSpy success(&transport, &SensorInterface::operationSucceeded);
    QSignalSpy failures(&transport, &SensorInterface::operationFailed);
    transport.connectSensor(port, 460800);
    QVERIFY(!connection.isEmpty() && connection.last()[0].toBool());
    transport.writeSettings(QString::fromUtf8(QJsonDocument(expected).toJson(QJsonDocument::Compact)));
    QVERIFY2(!success.isEmpty(), qPrintable(failures.isEmpty() ? "No completion signal" : failures.last()[1].toString()));
    QVERIFY(connection.last()[0].toBool());
    transport.disconnectSensor();
  }
};

QTEST_MAIN(HardwareIntegrationTest)
#include "test_hardware_integration.moc"
