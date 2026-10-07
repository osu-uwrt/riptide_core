#include <QtTest/QtTest>

#include <QComboBox>
#include <QDockWidget>
#include <QSettings>
#include <QTableWidget>
#include <QTreeWidget>

#include "riptide_imu/config_tool/dashboard_widgets.hpp"
#include "riptide_imu/config_tool/main_window.hpp"

namespace riptide_imu {

class FakeSensor : public SensorInterface {
  Q_OBJECT
 public:
  QString connected_port;
  int connected_baud{0};
  int connect_count{0};
  int disconnect_count{0};
  QVector<QPair<int, QString>> writes;

 public slots:
  void connectSensor(const QString& port, int baud) override {
    connected_port = port;
    connected_baud = baud;
    ++connect_count;
    emit connectionChanged(true, "Fake VN-100 connected");
  }
  void disconnectSensor() override {
    ++disconnect_count;
    emit connectionChanged(false, "Disconnected");
  }
  void setDemoMode(bool) override {}
  void readRegister(int id) override { emit registerRead(id, QString()); }
  void writeRegister(int id, const QString& values) override {
    writes.push_back({id, values});
    emit writeVerified(id, values);
  }
  void writeSettings() override {}
  void tare() override {}
  void reset() override {}

  void provideRegister(int id, const QString& value) { emit registerRead(id, value); }
};

class DashboardTest : public QObject {
  Q_OBJECT
 private slots:
  void initTestCase() {
    QSettings::setDefaultFormat(QSettings::IniFormat);
    QSettings::setPath(QSettings::IniFormat, QSettings::UserScope,
                       "/tmp/vn_dashboard_qsettings");
    QCoreApplication::setOrganizationName("UWRT Dashboard Test");
    QCoreApplication::setApplicationName("VectorNav Workbench Test");
    QSettings().clear();
    applyDarkTheme(qApp);
  }

  void init() { QSettings().clear(); }

  void darkThemeIsMandatory() {
    QVERIFY(qApp->property("vnDarkTheme").toBool());
    const auto palette = qApp->palette();
    QVERIFY(palette.color(QPalette::Window).lightness() <
            palette.color(QPalette::WindowText).lightness());
  }

  void dashboardUsesLaunchConnectionAndDockedWorkspace() {
    FakeSensor sensor;
    MainWindow window("/dev/launch-port", 230400, QString(), false,
                      QStringLiteral(TEST_SHARE_DIRECTORY), nullptr, &sensor);
    QCOMPARE(sensor.connect_count, 1);
    QCOMPARE(sensor.connected_port, QString("/dev/launch-port"));
    QCOMPARE(sensor.connected_baud, 230400);
    QCOMPARE(window.findChild<QComboBox*>("sensorPortSelector")->currentText(),
             QString("/dev/launch-port"));
    QVERIFY(window.findChild<QTreeWidget*>("registerTree"));
    QVERIFY(window.findChild<QTableWidget*>("registerInspector"));
    QVERIFY(window.findChild<QDockWidget*>("monitorDock"));
    QVERIFY(window.findChild<QDockWidget*>("plotDock"));
    QVERIFY(window.findChild<QDockWidget*>("calibrationDock"));
    QVERIFY(window.findChild<QDockWidget*>("diagnosticsDock"));
    QCOMPARE(window.size(), QSize(1440, 900));
  }

  void editablePortReconnectsWithoutRelaunch() {
    FakeSensor sensor;
    MainWindow window("/dev/initial", 460800, QString(), false,
                      QStringLiteral(TEST_SHARE_DIRECTORY), nullptr, &sensor);
    QVERIFY(QMetaObject::invokeMethod(&window, "toggleConnection"));
    QCOMPARE(sensor.disconnect_count, 1);
    auto* port = window.findChild<QComboBox*>("sensorPortSelector");
    auto* baud = window.findChild<QComboBox*>("sensorBaudSelector");
    port->setCurrentText("/dev/manual");
    baud->setCurrentText("115200");
    QVERIFY(QMetaObject::invokeMethod(&window, "toggleConnection"));
    QCOMPARE(sensor.connect_count, 2);
    QCOMPARE(sensor.connected_port, QString("/dev/manual"));
    QCOMPARE(sensor.connected_baud, 115200);
  }

  void structuredRegisterEditStagesAndApplies() {
    FakeSensor sensor;
    MainWindow window("/dev/test", 460800, QString(), false,
                      QStringLiteral(TEST_SHARE_DIRECTORY), nullptr, &sensor);
    sensor.provideRegister(35, "0;0;0;0");
    auto* tree = window.findChild<QTreeWidget*>("registerTree");
    QTreeWidgetItem* target = nullptr;
    for (int group = 0; group < tree->topLevelItemCount(); ++group) {
      auto* group_item = tree->topLevelItem(group);
      for (int row = 0; row < group_item->childCount(); ++row) {
        if (group_item->child(row)->data(0, Qt::UserRole).toInt() == 35)
          target = group_item->child(row);
      }
    }
    QVERIFY(target);
    tree->setCurrentItem(target);
    auto* inspector = window.findChild<QTableWidget*>("registerInspector");
    auto* enable = qobject_cast<QComboBox*>(inspector->cellWidget(0, 2));
    QVERIFY(enable);
    enable->setCurrentIndex(enable->findData("1"));
    QVERIFY(QMetaObject::invokeMethod(&window, "applyStagedRegisters"));
    QCOMPARE(sensor.writes.size(), 1);
    QCOMPARE(sensor.writes.front().first, 35);
    QCOMPARE(sensor.writes.front().second, QString("1;0;0;0"));
  }

  void layoutResetRestoresAllPanels() {
    FakeSensor sensor;
    MainWindow window("/dev/test", 460800, QString(), false,
                      QStringLiteral(TEST_SHARE_DIRECTORY), nullptr, &sensor);
    window.findChild<QDockWidget*>("monitorDock")->hide();
    QVERIFY(QMetaObject::invokeMethod(&window, "resetDashboardLayout"));
    QVERIFY(!window.findChild<QDockWidget*>("monitorDock")->isHidden());
    QCOMPARE(window.size(), QSize(1440, 900));
  }

  void dockVisibilityPersistsAndSmallerScreensRemainUsable() {
    FakeSensor sensor;
    {
      MainWindow first("/dev/test", 460800, QString(), false,
                       QStringLiteral(TEST_SHARE_DIRECTORY), nullptr, &sensor);
      first.resize(1280, 720);
      first.show();
      QTest::qWait(20);
      first.findChild<QDockWidget*>("monitorDock")->hide();
      QCoreApplication::processEvents();
    }
    {
      QSettings persisted;
      persisted.sync();
      QVERIFY(persisted.contains("dashboard/docks/monitorDock/visible"));
      MainWindow restored("/dev/test", 460800, QString(), false,
                          QStringLiteral(TEST_SHARE_DIRECTORY), nullptr, &sensor);
      QCOMPARE(persisted.value("dashboard/docks/monitorDock/visible").toBool(), false);
      restored.show();
      QTest::qWait(20);
      QVERIFY(!restored.findChild<QDockWidget*>("monitorDock")->isVisible());
      QVERIFY(restored.findChild<QTreeWidget*>("registerTree")->isEnabled());
      QVERIFY(restored.findChild<QComboBox*>("sensorPortSelector"));
      QCOMPARE(restored.size(), QSize(1280, 720));
    }
  }

  void rendersTargetDashboardForVisualInspection() {
    FakeSensor sensor;
    MainWindow window("/dev/demo", 460800, QString(), false,
                      QStringLiteral(TEST_SHARE_DIRECTORY), nullptr, &sensor);
    window.resize(1440, 900);
    window.show();
    QTest::qWait(50);
    const QPixmap image = window.grab();
    QCOMPARE(image.size(), QSize(1440, 900));
    QVERIFY(image.save("/tmp/vn_dashboard_1440x900.png"));
  }
};

}  // namespace riptide_imu

QTEST_MAIN(riptide_imu::DashboardTest)
#include "test_dashboard.moc"
