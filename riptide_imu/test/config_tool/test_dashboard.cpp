#include <QtTest/QtTest>

#include <QAbstractItemView>
#include <QCheckBox>
#include <QComboBox>
#include <QCompleter>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QMouseEvent>
#include <QPushButton>
#include <QSettings>
#include <QSplitter>
#include <QStackedWidget>
#include <QTableWidget>
#include <QTemporaryDir>
#include <QTimer>
#include <QTreeWidget>
#include <cmath>

#include "riptide_imu/config_tool/calibration_wizard.hpp"
#include "riptide_imu/config_tool/dashboard_widgets.hpp"
#include "riptide_imu/config_tool/main_window.hpp"
#include "riptide_imu/config_tool/theme.hpp"

namespace riptide_imu {

class FakeSensor : public SensorInterface {
  Q_OBJECT
 public:
  QString connected_port;
  int connected_baud{0};
  int connect_count{0};
  int disconnect_count{0};
  QString connection_mode;
  int discovery_count{0};
  QVector<QPair<int, QString>> writes;
  QVector<int> reads;
  QMap<int, QString> registers;  // what reads return; empty for any other register
  QStringList endpoints{"/talos"};
  bool capture{false};
  int reset_count{0};
  int maintenance_requests{0};

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
  void setConnectionMode(const QString& mode) override { connection_mode = mode; }
  void discoverEndpoints() override {
    ++discovery_count;
    emit endpointsDiscovered(endpoints);
  }
  void setDemoMode(bool) override {}
  void setCalibrationCapture(bool enabled) override { capture = enabled; }
  void readRegister(int id) override {
    reads.push_back(id);
    emit registerRead(id, registers.value(id));
  }
  void writeRegister(int id, const QString& values) override {
    writes.push_back({id, values});
    registers[id] = values;
    emit writeVerified(id, values);
  }
  void writeSettings(const QString&) override {}
  void tare() override {}
  void reset() override { ++reset_count; }
  void openMaintenance() override {
    ++maintenance_requests;
    SensorInterface::openMaintenance();
  }

  void provideRegister(int id, const QString& value) { emit registerRead(id, value); }
  void provideCalibration(const Telemetry& value) { emit calibrationSampleReceived(value); }
  void provideTelemetry(const Telemetry& value) { emit telemetryReceived(value); }
};

namespace {

QTreeWidgetItem* registerItem(QTreeWidget* tree, int id) {
  for (int group = 0; group < tree->topLevelItemCount(); ++group) {
    auto* group_item = tree->topLevelItem(group);
    for (int row = 0; row < group_item->childCount(); ++row)
      if (group_item->child(row)->data(0, Qt::UserRole).toInt() == id) return group_item->child(row);
  }
  return nullptr;
}

// Accepts the next message box (warnings ask before anything is written).
void acceptNextMessageBox() {
  QTimer::singleShot(0, [] {
    for (QWidget* widget : QApplication::topLevelWidgets())
      if (auto* box = qobject_cast<QMessageBox*>(widget)) box->accept();
  });
}

// Clicks the button with this text in the next message box that has one.
void clickInMessageBox(const QString& text) {
  auto* timer = new QTimer;
  timer->setInterval(20);
  QObject::connect(timer, &QTimer::timeout, [timer, text] {
    for (QWidget* widget : QApplication::topLevelWidgets()) {
      auto* box = qobject_cast<QMessageBox*>(widget);
      if (!box || !box->isVisible()) continue;
      for (auto* button : box->buttons()) {
        if (button->text().remove('&') != text) continue;
        timer->stop();
        timer->deleteLater();
        button->click();
        return;
      }
    }
  });
  timer->start();
}

const QMap<int, QString> kTalosOutput{{5, "460800"}, {6, "0"}, {7, "50"}, {75, "1;2;03;0530;0001"},
                                     {76, "0;0;00"}, {77, "0;0;00"}};

// A resting VN-100 reading at `time_s`: gravity along `down` in the sensor frame, a small gyro bias and noise.
Telemetry resting(double time_s, const Eigen::Vector3d& down = Eigen::Vector3d::UnitZ(), int index = 0) {
  Telemetry sample;
  sample.time_s = time_s;
  const double wobble = 0.0004 * std::sin(index * 1.7);
  const Eigen::Vector3d force = -9.8 * down.normalized();
  sample.ax = force.x() + 0.002 * std::sin(index * 2.3) + 0.04;
  sample.ay = force.y() + 0.002 * std::cos(index * 1.1) - 0.03;
  sample.az = force.z() + 0.025;
  sample.gx = 0.0012 + 0.0001 * std::sin(index * 0.9);
  sample.gy = -0.0008 + 0.0001 * std::cos(index * 1.3);
  sample.gz = 0.00045 + wobble * 0.1;
  sample.mx = 0.22;
  sample.my = 0.03;
  sample.mz = 0.41;
  sample.temperature_c = 30.0;
  return sample;
}

}  // namespace

class DashboardTest : public QObject {
  Q_OBJECT
 private slots:
  void initTestCase() {
    QSettings::setDefaultFormat(QSettings::IniFormat);
    QSettings::setPath(QSettings::IniFormat, QSettings::UserScope, "/tmp/vn_dashboard_qsettings");
    QCoreApplication::setOrganizationName("UWRT Dashboard Test");
    QCoreApplication::setApplicationName("VectorNav Workbench Test");
    QSettings().clear();
    applyTheme(qApp, "heat-sheet", QStringLiteral(TEST_SHARE_DIRECTORY));
  }

  void init() {
    QSettings().clear();
    applyTheme(qApp, "heat-sheet", QStringLiteral(TEST_SHARE_DIRECTORY));
  }

  void themesFollowNereusWithTimingBoardDefault() {
    QCOMPARE(qApp->property("vnTheme").toString(), QString("heat-sheet"));
    QVERIFY(qApp->palette().color(QPalette::Window).lightness() >
            qApp->palette().color(QPalette::WindowText).lightness());
    QVERIFY(QFontInfo(uiFont()).family().startsWith("Barlow"));
    applyTheme(qApp, "timing-board", QStringLiteral(TEST_SHARE_DIRECTORY));
    QVERIFY(theme().dark);
    QVERIFY(qApp->palette().color(QPalette::Window).lightness() <
            qApp->palette().color(QPalette::WindowText).lightness());
    applyTheme(qApp, "no-such-theme");
    QCOMPARE(theme().id, QString("timing-board"));
    QCOMPARE(themeIds().front(), QString("timing-board"));
  }

  void dashboardUsesLaunchConnectionAndPanels() {
    FakeSensor sensor;
    MainWindow window("/dev/launch-port", 230400, QString(), false, QStringLiteral(TEST_SHARE_DIRECTORY),
                      nullptr, &sensor);
    QCOMPARE(sensor.connect_count, 1);
    QCOMPARE(sensor.connected_port, QString("/dev/launch-port"));
    QCOMPARE(sensor.connected_baud, 230400);
    QCOMPARE(window.findChild<QComboBox*>("sensorPortSelector")->currentText(), QString("/dev/launch-port"));
    for (const char* name : {"registerTree", "registerInspector", "attitudeView", "liveData", "signalPlot",
                             "calibrationSummary", "protocolLog", "mainSplitter"})
      QVERIFY2(window.findChild<QWidget*>(name), name);
    QCOMPARE(window.size(), QSize(1440, 900));
  }

  void editablePortReconnectsWithoutRelaunch() {
    FakeSensor sensor;
    MainWindow window("/dev/initial", 460800, QString(), false, QStringLiteral(TEST_SHARE_DIRECTORY), nullptr,
                      &sensor);
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

  void rosModeIsDefaultCapableAndUsesDiscoveredRobotEndpoint() {
    FakeSensor sensor;
    MainWindow window(QString(), 460800, QString(), false, QStringLiteral(TEST_SHARE_DIRECTORY), nullptr,
                      &sensor, "ros");
    QCOMPARE(sensor.connection_mode, QString("ros"));
    QCOMPARE(sensor.discovery_count, 1);
    QCOMPARE(sensor.connected_port, QString("/talos"));
    QCOMPARE(window.findChild<SegmentedSwitch*>("connectionModeSelector")->currentIndex(), 0);
    QCOMPARE(window.findChild<QComboBox*>("sensorPortSelector")->currentText(), QString("/talos"));
  }

  void structuredRegisterEditStagesAndApplies() {
    FakeSensor sensor;
    MainWindow window("/dev/test", 460800, QString(), false, QStringLiteral(TEST_SHARE_DIRECTORY), nullptr,
                      &sensor);
    sensor.provideRegister(35, "0;0;0;0");
    auto* tree = window.findChild<QTreeWidget*>("registerTree");
    QTreeWidgetItem* target = nullptr;
    for (int group = 0; group < tree->topLevelItemCount(); ++group) {
      auto* group_item = tree->topLevelItem(group);
      for (int row = 0; row < group_item->childCount(); ++row)
        if (group_item->child(row)->data(0, Qt::UserRole).toInt() == 35) target = group_item->child(row);
    }
    QVERIFY(target);
    tree->setCurrentItem(target);
    auto* inspector = window.findChild<QTableWidget*>("registerInspector");
    auto* enable = qobject_cast<QComboBox*>(inspector->cellWidget(0, 2));
    QVERIFY(enable);
    enable->setCurrentIndex(enable->findData("1"));
    QCOMPARE(window.findChild<StatusChip*>("stagedChip")->text(), QString("1 staged"));
    QCOMPARE(window.findChild<QPushButton*>("applyButton")->text(), QString("Apply 1"));
    QVERIFY(QMetaObject::invokeMethod(&window, "applyStagedRegisters"));
    QCOMPARE(sensor.writes.size(), 1);
    QCOMPARE(sensor.writes.front().first, 35);
    QCOMPARE(sensor.writes.front().second, QString("1;0;0;0"));
    QCOMPARE(window.findChild<QPushButton*>("applyButton")->text(), QString("Apply"));
  }

  void layoutPersistsAndResets() {
    FakeSensor sensor;
    QList<int> saved;
    {
      MainWindow first("/dev/test", 460800, QString(), false, QStringLiteral(TEST_SHARE_DIRECTORY), nullptr,
                       &sensor);
      first.resize(1280, 720);
      first.show();
      QTest::qWait(20);
      auto* splitter = first.findChild<QSplitter*>("mainSplitter");
      splitter->setSizes({260, 700, 308});
      saved = splitter->sizes();
      QVERIFY(saved.front() < 330);
    }
    MainWindow restored("/dev/test", 460800, QString(), false, QStringLiteral(TEST_SHARE_DIRECTORY), nullptr,
                        &sensor);
    restored.show();
    QTest::qWait(20);
    QCOMPARE(restored.size(), QSize(1280, 720));
    QCOMPARE(restored.findChild<QSplitter*>("mainSplitter")->sizes(), saved);
    QVERIFY(QMetaObject::invokeMethod(&restored, "resetDashboardLayout"));
    QCOMPARE(restored.size(), QSize(1440, 900));
    QVERIFY(!QSettings().contains("workbench/main"));
  }

  void attitudeViewReadsBothFrames() {
    AttitudeWidget view;
    // 30 degrees of yaw from NED (sensor quaternion).
    const Eigen::Quaterniond yaw(Eigen::AngleAxisd(M_PI / 6, Eigen::Vector3d::UnitZ()));
    view.setQuaternion(yaw.x(), yaw.y(), yaw.z(), yaw.w());
    QVERIFY(view.eulerDegrees().isApprox(Eigen::Vector3d(0, 0, 30), 1e-6));
    // The ROS driver publishes the same attitude relative to ENU: q_enu = ned_to_enu * q_ned.
    view.setWorldFrame(AttitudeWidget::WorldFrame::Enu);
    const Eigen::Quaterniond enu(Eigen::Matrix3d(nedToEnu() * yaw.toRotationMatrix()));
    view.setQuaternion(enu.x(), enu.y(), enu.z(), enu.w());
    QVERIFY(view.eulerDegrees().isApprox(Eigen::Vector3d(0, 0, 30), 1e-6));
  }

  void wizardGyroHoldCompletesAndRestartsOnMotion() {
    FakeSensor sensor;
    MainWindow window("/dev/test", 460800, QString(), false, QStringLiteral(TEST_SHARE_DIRECTORY), nullptr,
                      &sensor);
    window.openCalibrationWizard();
    auto* wizard = window.calibrationWizard();
    QVERIFY(wizard);
    QVERIFY(sensor.capture);
    for (int id : {23, 25, 26, 43, 84}) QVERIFY(sensor.reads.contains(id));
    wizard->goToPage(CalibrationWizard::Gyro);
    double t = 100.0;
    int index = 0;
    sensor.provideCalibration(resting(t, Eigen::Vector3d::UnitZ(), index++));
    wizard->startCapture();
    for (; t < 110.0; t += 0.05) sensor.provideCalibration(resting(t, Eigen::Vector3d::UnitZ(), index++));
    QCOMPARE(wizard->stepState(CalibrationStep::Gyro), CalibrationWizard::StepState::Capturing);
    QVERIFY(window.calibrationSession().gyro.size() > 150);
    // A bump discards the hold.
    Telemetry bump = resting(t, Eigen::Vector3d::UnitZ(), index++);
    bump.gx += 0.08;
    sensor.provideCalibration(bump);
    QVERIFY(window.calibrationSession().gyro.empty());
    for (t += 0.05; t < 145.0; t += 0.05) sensor.provideCalibration(resting(t, Eigen::Vector3d::UnitZ(), index++));
    QCOMPARE(wizard->stepState(CalibrationStep::Gyro), CalibrationWizard::StepState::Passed);
    QVERIFY(window.calibrationSession().gyro_result.residual_bias.isApprox(Eigen::Vector3d(0.0012, -0.0008, 0.00045),
                                                                          1e-2));
    wizard->close();
    QVERIFY(!sensor.capture);
  }

  void wizardAccelerometerCapturesEveryPoseAutomatically() {
    FakeSensor sensor;
    MainWindow window("/dev/test", 460800, QString(), false, QStringLiteral(TEST_SHARE_DIRECTORY), nullptr,
                      &sensor);
    window.openCalibrationWizard();
    auto* wizard = window.calibrationWizard();
    wizard->goToPage(CalibrationWizard::Accelerometer);
    double t = 10.0;
    int index = 0;
    sensor.provideCalibration(resting(t));
    wizard->startCapture();
    for (const auto& pose : accelerometerPoses()) {
      // Carry it to the pose (moving), then let it rest there for 4 s.
      for (int i = 0; i < 10; ++i, t += 0.05) {
        Telemetry moving = resting(t, pose.direction(), index++);
        moving.gy += 0.3;
        sensor.provideCalibration(moving);
      }
      const double settle_until = t + 4.0;
      for (; t < settle_until; t += 0.05) sensor.provideCalibration(resting(t, pose.direction(), index++));
    }
    QCOMPARE(wizard->stepState(CalibrationStep::Accelerometer), CalibrationWizard::StepState::Passed);
    const auto& result = window.calibrationSession().accelerometer_result;
    QVERIFY2(result.quality.passed, result.quality.summary.c_str());
    QVERIFY((result.bias - Eigen::Vector3d(0.04, -0.03, 0.025)).norm() < 0.01);
  }

  void wizardMagnetometerTurnFillsCoverage() {
    FakeSensor sensor;
    MainWindow window("/dev/test", 460800, QString(), false, QStringLiteral(TEST_SHARE_DIRECTORY), nullptr,
                      &sensor);
    window.openCalibrationWizard();
    auto* wizard = window.calibrationWizard();
    wizard->goToPage(CalibrationWizard::Magnetometer);
    const Eigen::Vector3d hard_iron(0.07, -0.045, 0.09);
    const double rate = 15.0 * M_PI / 180.0;
    double t = 50.0;
    sensor.provideCalibration(resting(t));
    wizard->startCapture();
    for (int i = 0; i < 520 && wizard->stepState(CalibrationStep::Magnetometer) ==
                                   CalibrationWizard::StepState::Capturing; ++i) {
      t += 0.05;
      const double yaw = rate * i * 0.05;
      Telemetry sample = resting(t, Eigen::Vector3d::UnitZ(), i);
      sample.gz = rate;
      const Eigen::Vector3d field =
          Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()) * Eigen::Vector3d(0.22, 0.03, 0.41) + hard_iron;
      sample.mx = field.x();
      sample.my = field.y();
      sample.mz = field.z();
      sensor.provideCalibration(sample);
    }
    QCOMPARE(wizard->stepState(CalibrationStep::Magnetometer), CalibrationWizard::StepState::Passed);
    const auto& result = window.calibrationSession().magnetometer_result;
    QVERIFY(result.coverage_deg > 340);
    QVERIFY((result.in_plane_bias.head<2>() - hard_iron.head<2>()).norm() < 0.01);
  }

  void wizardStagesPassedGyroIntoRegisterWorkspace() {
    FakeSensor sensor;
    MainWindow window("/dev/test", 460800, QString(), false, QStringLiteral(TEST_SHARE_DIRECTORY), nullptr,
                      &sensor);
    window.openCalibrationWizard();
    sensor.provideRegister(84, "1;0;0;0;1;0;0;0;1;0;0;0");
    sensor.provideRegister(26, "1;0;0;0;1;0;0;0;1");
    sensor.provideRegister(43, "0;0;0");
    auto* wizard = window.calibrationWizard();
    wizard->goToPage(CalibrationWizard::Gyro);
    double t = 1.0;
    int index = 0;
    sensor.provideCalibration(resting(t));
    wizard->startCapture();
    for (; t < 34.0; t += 0.05) sensor.provideCalibration(resting(t, Eigen::Vector3d::UnitZ(), index++));
    QCOMPARE(wizard->stepState(CalibrationStep::Gyro), CalibrationWizard::StepState::Passed);
    wizard->goToPage(CalibrationWizard::Review);
    auto* stage = wizard->findChild<QPushButton*>("stageButton");
    QVERIFY(stage && stage->isEnabled());
    stage->click();
    QCOMPARE(wizard->stepState(CalibrationStep::Gyro), CalibrationWizard::StepState::Staged);
    QCOMPARE(window.findChild<StatusChip*>("stagedChip")->text(), QString("1 staged"));
    QVERIFY(sensor.writes.isEmpty());
  }

  void connectingReadsOutputConfigAndShowsLineLoad() {
    FakeSensor sensor;
    sensor.registers = kTalosOutput;
    MainWindow window("/dev/test", 460800, QString(), false, QStringLiteral(TEST_SHARE_DIRECTORY), nullptr,
                      &sensor);
    for (int id : {5, 6, 7, 75, 76, 77}) QVERIFY(sensor.reads.contains(id));
    const auto load = window.bandwidth(false);
    QVERIFY(load.known);
    QCOMPARE(qRound(100 * load.fraction()), 66);
    QVERIFY(window.findChild<BandwidthMeter*>("bandwidthMeter")->isVisibleTo(&window));
    // The board names the sensor's own rate, not this view's polling rate.
    QVERIFY(window.findChild<StatusChip*>("telemetryChip")->toolTip().contains("400 Hz"));
  }

  void selectingARegisterReadsItExceptTheResettingOne() {
    FakeSensor sensor;
    MainWindow window("/dev/test", 460800, QString(), false, QStringLiteral(TEST_SHARE_DIRECTORY), nullptr,
                      &sensor);
    // The register selected when the connection comes up (User Tag) is read.
    QTRY_VERIFY(sensor.reads.contains(0));
    auto* tree = window.findChild<QTreeWidget*>("registerTree");
    sensor.reads.clear();
    tree->setCurrentItem(registerItem(tree, 54));
    QTRY_VERIFY(sensor.reads.contains(54));
    // Clicking it again reads it again.
    sensor.reads.clear();
    emit tree->itemClicked(registerItem(tree, 54), 0);
    QTRY_VERIFY(sensor.reads.contains(54));
    sensor.reads.clear();
    tree->setCurrentItem(registerItem(tree, 80));
    QTest::qWait(300);
    QVERIFY(!sensor.reads.contains(80));
    // A register with a staged change is not read over.
    sensor.provideRegister(35, "0;0;0;0");
    tree->setCurrentItem(registerItem(tree, 35));
    QTRY_VERIFY(sensor.reads.contains(35));
    auto* enable = qobject_cast<QComboBox*>(window.findChild<QTableWidget*>("registerInspector")->cellWidget(0, 2));
    enable->setCurrentIndex(enable->findData("1"));
    QVERIFY(window.stagedValues().contains(35));
    sensor.reads.clear();
    emit tree->itemClicked(registerItem(tree, 35), 0);
    QTest::qWait(300);
    QVERIFY(!sensor.reads.contains(35));
  }

  void binaryOutputEditsAsCheckboxesAndOverBudgetIsBlocked() {
    FakeSensor sensor;
    sensor.registers = kTalosOutput;
    MainWindow window("/dev/test", 460800, QString(), false, QStringLiteral(TEST_SHARE_DIRECTORY), nullptr,
                      &sensor);
    auto* tree = window.findChild<QTreeWidget*>("registerTree");
    tree->setCurrentItem(registerItem(tree, 75));
    auto* editor = window.findChild<BinaryOutputEditor*>("binaryOutputEditor");
    QVERIFY(editor && editor->isVisibleTo(&window));
    QCheckBox* dcm = nullptr;
    QCheckBox* quaternion = nullptr;
    for (auto* box : editor->findChildren<QCheckBox*>()) {
      if (box->toolTip().startsWith("DCM")) dcm = box;
      if (box->toolTip().startsWith("Quaternion (Common")) quaternion = box;
    }
    QVERIFY(dcm && quaternion);
    QVERIFY(quaternion->isChecked());
    QVERIFY(!dcm->isChecked());
    dcm->setChecked(true);
    QCOMPARE(window.stagedValues().value(75), QString("1;2;13;0530;0001;0008"));
    // 400 Hz with the DCM still fits; at 800 Hz it does not, and Apply refuses.
    auto* rate = editor->findChild<QComboBox*>("binaryOutputRate");
    rate->setCurrentIndex(rate->findData(1));
    QVERIFY(window.bandwidth(true).exceeds());
    acceptNextMessageBox();
    QVERIFY(QMetaObject::invokeMethod(&window, "applyStagedRegisters"));
    QVERIFY(sensor.writes.isEmpty());
  }

  void discardedCalibrationCanBeStagedAgain() {
    FakeSensor sensor;
    sensor.registers = {{84, "1;0;0;0;1;0;0;0;1;0;0;0"}, {26, "1;0;0;0;1;0;0;0;1"}, {43, "0;0;0"}};
    MainWindow window("/dev/test", 460800, QString(), false, QStringLiteral(TEST_SHARE_DIRECTORY), nullptr,
                      &sensor);
    window.openCalibrationWizard();
    auto* wizard = window.calibrationWizard();
    wizard->goToPage(CalibrationWizard::Gyro);
    double t = 1.0;
    int index = 0;
    sensor.provideCalibration(resting(t));
    wizard->startCapture();
    for (; t < 34.0; t += 0.05) sensor.provideCalibration(resting(t, Eigen::Vector3d::UnitZ(), index++));
    QCOMPARE(wizard->stepState(CalibrationStep::Gyro), CalibrationWizard::StepState::Passed);
    auto* stage_latest = window.findChild<QPushButton*>("stageLatestButton");
    QVERIFY(stage_latest->isEnabled());
    stage_latest->click();
    QCOMPARE(wizard->stepState(CalibrationStep::Gyro), CalibrationWizard::StepState::Staged);
    QVERIFY(!stage_latest->isEnabled());
    // Discarding takes it out of the workspace, and nothing claims otherwise.
    QVERIFY(QMetaObject::invokeMethod(&window, "discardStagedRegisters"));
    QCOMPARE(wizard->stepState(CalibrationStep::Gyro), CalibrationWizard::StepState::Passed);
    auto* summary = window.findChild<MetricTable*>("calibrationSummary");
    QCOMPARE(summary->rows().front().verdict, QString("Passed"));
    QVERIFY(stage_latest->isEnabled());
    stage_latest->click();
    QCOMPARE(window.findChild<StatusChip*>("stagedChip")->text(), QString("1 staged"));
    // Applied: the sensor echoes fewer digits, and it still counts as the same value.
    QVERIFY(QMetaObject::invokeMethod(&window, "applyStagedRegisters"));
    QCOMPARE(wizard->stepState(CalibrationStep::Gyro), CalibrationWizard::StepState::Applied);
    QCOMPARE(summary->rows().front().verdict, QString("On sensor"));
  }

  void orbitingSwitchesThePresetTo3d() {
    FakeSensor sensor;
    MainWindow window("/dev/test", 460800, QString(), false, QStringLiteral(TEST_SHARE_DIRECTORY), nullptr,
                      &sensor);
    auto* preset = window.findChild<SegmentedSwitch*>("attitudeViewPreset");
    auto* view = window.findChild<AttitudeWidget*>("attitudeView");
    preset->setCurrentIndex(1);
    QMouseEvent press(QEvent::MouseButtonPress, QPointF(100, 100), Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
    QMouseEvent drag(QEvent::MouseMove, QPointF(140, 120), Qt::NoButton, Qt::LeftButton, Qt::NoModifier);
    QCoreApplication::sendEvent(view, &press);
    QCoreApplication::sendEvent(view, &drag);
    QCOMPARE(preset->currentIndex(), 0);
  }

  void targetListRefreshesOnClickAndFilters() {
    FakeSensor sensor;
    MainWindow window(QString(), 460800, QString(), false, QStringLiteral(TEST_SHARE_DIRECTORY), nullptr, &sensor,
                      "ros");
    window.show();
    QCOMPARE(sensor.connect_count, 1);  // the one robot found at start
    QVERIFY(QMetaObject::invokeMethod(&window, "toggleConnection"));  // disconnect, so the box is editable
    auto* port = window.findChild<QComboBox*>("sensorPortSelector");
    QCOMPARE(port->completer()->filterMode(), Qt::MatchContains);
    // Two more robots come up; a click finds them without pressing Rescan, and does not connect.
    sensor.endpoints = QStringList{"/mercury", "/puddles", "/talos"};
    const int discoveries = sensor.discovery_count;
    QTest::mouseClick(port->lineEdit(), Qt::LeftButton);
    QCOMPARE(sensor.discovery_count, discoveries + 1);
    QCOMPARE(port->count(), 3);
    QTRY_VERIFY(port->completer()->popup()->isVisible());
    QCOMPARE(port->completer()->completionCount(), 3);
    port->completer()->setCompletionPrefix("merc");
    QCOMPARE(port->completer()->completionCount(), 1);
    // Down to one robot: a click refresh still only lists it.
    port->completer()->popup()->hide();
    sensor.endpoints = QStringList{"/talos"};
    QTest::mouseClick(port->lineEdit(), Qt::LeftButton);
    QCOMPARE(port->count(), 1);
    QCOMPARE(sensor.connect_count, 1);
  }

  void plotShowsAttitudeAndHidesAxes() {
    FakeSensor sensor;
    MainWindow window("/dev/test", 460800, QString(), false, QStringLiteral(TEST_SHARE_DIRECTORY), nullptr,
                      &sensor);
    auto* plot = window.findChild<PlotWidget*>("signalPlot");
    QCOMPARE(plot->quantity(), int(PlotWidget::Attitude));
    QCOMPARE(PlotWidget::axisNames(plot->quantity()).value(2), QString("Yaw"));
    window.resize(1440, 900);
    window.show();
    QVERIFY(QTest::qWaitForWindowExposed(&window));
    QVERIFY(plot->isVisible());
    plot->repaint();
    // the legend entries are the toggles; a hidden one stays in place to click back on
    const QPoint roll = plot->legendRect(0).center().toPoint();
    QTest::mouseClick(plot, Qt::LeftButton, Qt::NoModifier, roll);
    QVERIFY(!plot->axisVisible(0));
    QVERIFY(plot->axisVisible(2));
    plot->repaint();
    QCOMPARE(plot->legendRect(0).center().toPoint(), roll);
    QTest::mouseClick(plot, Qt::LeftButton, Qt::NoModifier, roll);
    QVERIFY(plot->axisVisible(0));
  }

  void maintenanceIsAskedForThenTheActionContinues() {
    FakeSensor sensor;
    MainWindow window(QString(), 460800, QString(), false, QStringLiteral(TEST_SHARE_DIRECTORY), nullptr, &sensor,
                      "ros");
    window.show();
    // declining leaves the sensor alone
    clickInMessageBox("Cancel");
    QVERIFY(QMetaObject::invokeMethod(&window, "resetSensor"));
    QCOMPARE(sensor.maintenance_requests, 0);
    QCOMPARE(sensor.reset_count, 0);
    // entering it carries on to the reset's own confirmation
    clickInMessageBox("Enter maintenance and continue");
    clickInMessageBox("Yes");
    QVERIFY(QMetaObject::invokeMethod(&window, "resetSensor"));
    QCOMPARE(sensor.maintenance_requests, 1);
    QTRY_COMPARE(sensor.reset_count, 1);
    QVERIFY(window.findChild<QPushButton*>("maintenanceButton")->text() == "Leave maintenance");
    // already in maintenance: no second ask
    clickInMessageBox("Yes");
    QVERIFY(QMetaObject::invokeMethod(&window, "resetSensor"));
    QTRY_COMPARE(sensor.reset_count, 2);
    QCOMPARE(sensor.maintenance_requests, 1);
  }

  void wizardTemperatureSaysChangingAndHoldsItsVerdict() {
    FakeSensor sensor;
    MainWindow window("/dev/test", 460800, QString(), false, QStringLiteral(TEST_SHARE_DIRECTORY), nullptr,
                      &sensor);
    window.openCalibrationWizard();
    auto* wizard = window.calibrationWizard();
    // Cooling at 1 C/min for a minute.
    double t = 0.0;
    for (int i = 0; t < 60.0; ++i, t += 0.1) {
      Telemetry sample = resting(t, Eigen::Vector3d::UnitZ(), i);
      sample.temperature_c = 35.0 - t / 60.0;
      sensor.provideCalibration(sample);
    }
    StatusChip* chip = nullptr;
    for (auto* candidate : wizard->findChildren<StatusChip*>())
      if (candidate->text() == "Steady" || candidate->text() == "Changing") chip = candidate;
    QVERIFY(chip);
    QCOMPARE(chip->text(), QString("Changing"));
    // It levels off: the verdict stays put for a while before it flips.
    for (int i = 0; t < 140.0; ++i, t += 0.1) {
      Telemetry sample = resting(t, Eigen::Vector3d::UnitZ(), i);
      sample.temperature_c = 34.0;
      sensor.provideCalibration(sample);
      if (t < 100.0) QCOMPARE(chip->text(), QString("Changing"));
    }
    QCOMPARE(chip->text(), QString("Steady"));
  }

  void wizardMagnetometerStartsAtZeroCoverage() {
    FakeSensor sensor;
    MainWindow window("/dev/test", 460800, QString(), false, QStringLiteral(TEST_SHARE_DIRECTORY), nullptr,
                      &sensor);
    window.openCalibrationWizard();
    auto* wizard = window.calibrationWizard();
    wizard->goToPage(CalibrationWizard::Magnetometer);
    double t = 10.0;
    int index = 0;
    for (; t < 12.0; t += 0.05) sensor.provideCalibration(resting(t, Eigen::Vector3d::UnitZ(), index++));
    wizard->startCapture();
    for (; t < 22.0; t += 0.05) sensor.provideCalibration(resting(t, Eigen::Vector3d::UnitZ(), index++));
    bool checked = false;
    for (auto* table : wizard->findChildren<MetricTable*>())
      for (const auto& row : table->rows())
        if (row.label == "Heading covered") {
          QCOMPARE(row.value, QString("0°"));
          checked = true;
        }
    QVERIFY(checked);
  }

  void wizardUsesARobotConfigForDirectionsAndTheGravityReport() {
    QTemporaryDir directory;
    QFile file(directory.filePath("talos.yaml"));
    QVERIFY(file.open(QIODevice::WriteOnly));
    file.write("base_link: [-0.14, 0.03, -0.09]\nimu:\n  pose: [-0.25, -0.1492, 0.0222, 3.141, 0.0, -1.5707]\n");
    file.close();
    FakeSensor sensor;
    MainWindow window("/dev/test", 460800, QString(), false, QStringLiteral(TEST_SHARE_DIRECTORY), nullptr,
                      &sensor);
    window.openCalibrationWizard();
    auto* wizard = window.calibrationWizard();
    QVERIFY(wizard->useRobotConfig(file.fileName()));
    QVERIFY(wizard->guideFrame().robot);
    QVERIFY(QSettings().value("calibration/robots").toStringList().contains(file.fileName()));
    wizard->goToPage(CalibrationWizard::Gravity);
    double t = 1.0;
    int index = 0;
    sensor.provideCalibration(resting(t));
    wizard->startCapture();
    for (; t < 14.0; t += 0.05) sensor.provideCalibration(resting(t, Eigen::Vector3d::UnitZ(), index++));
    QCOMPARE(wizard->stepState(CalibrationStep::Gravity), CalibrationWizard::StepState::Passed);
    bool pose_line = false;
    for (auto* label : wizard->findChildren<QLabel*>())
      pose_line |= label->isVisibleTo(wizard) && label->text().contains("pose: [-0.25, -0.1492, 0.0222,");
    QVERIFY(pose_line);
    // Remembered for next time.
    wizard->close();
    FakeSensor other;
    MainWindow again("/dev/test", 460800, QString(), false, QStringLiteral(TEST_SHARE_DIRECTORY), nullptr, &other);
    again.openCalibrationWizard();
    QVERIFY(again.calibrationWizard()->robotMount());
    QCOMPARE(again.calibrationWizard()->robotMount()->name, QString("talos"));
  }

  void rendersDashboardAndWizardForVisualInspection() {
    for (const QString id : {"heat-sheet", "timing-board"}) {
      applyTheme(qApp, id, QStringLiteral(TEST_SHARE_DIRECTORY));
      FakeSensor sensor;
      MainWindow window("/dev/demo", 460800, QString(), false, QStringLiteral(TEST_SHARE_DIRECTORY), nullptr,
                        &sensor);
      window.resize(1440, 900);
      window.show();
      sensor.provideRegister(25, "1;0;0;0;1;0;0;0;1;0;0;0");
      const Eigen::Quaterniond q = Eigen::AngleAxisd(0.6, Eigen::Vector3d::UnitZ()) *
                                   Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitY()) *
                                   Eigen::AngleAxisd(-0.3, Eigen::Vector3d::UnitX());
      for (int i = 0; i < 200; ++i) {
        Telemetry sample = resting(1000 + i * 0.05, Eigen::Vector3d(0.25, -0.3, 1), i);
        sample.qx = q.x(), sample.qy = q.y(), sample.qz = q.z(), sample.qw = q.w();
        sample.gx = 0.05 * std::sin(i * 0.1);
        sample.gy = 0.03 * std::cos(i * 0.07);
        sensor.provideTelemetry(sample);
      }
      QTest::qWait(50);
      const QPixmap image = window.grab();
      QCOMPARE(image.size(), QSize(1440, 900));
      QVERIFY(image.save(QString("/tmp/vn_dashboard_%1.png").arg(id)));
      // Grazing view (Level preset), where lighting used to darken whole faces.
      AttitudeWidget level;
      level.resize(520, 420);
      level.setViewPreset(1);
      const Eigen::Quaterniond tilt(Eigen::AngleAxisd(0.12, Eigen::Vector3d::UnitX()));
      level.setQuaternion(tilt.x(), tilt.y(), tilt.z(), tilt.w());
      QVERIFY(level.grab().save(QString("/tmp/vn_attitude_level_%1.png").arg(id)));
      // Low view onto the connector end, as reported: the top face recedes steeply here.
      AttitudeWidget low;
      low.resize(520, 420);
      low.setQuaternion(0, 0, std::sin(-1.1), std::cos(-1.1));
      QMouseEvent press(QEvent::MouseButtonPress, QPointF(260, 200), Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
      QMouseEvent drag(QEvent::MouseMove, QPointF(260, 160), Qt::NoButton, Qt::LeftButton, Qt::NoModifier);
      QCoreApplication::sendEvent(&low, &press);
      QCoreApplication::sendEvent(&low, &drag);
      QVERIFY(low.grab().save(QString("/tmp/vn_attitude_low_%1.png").arg(id)));

      window.openCalibrationWizard();
      auto* wizard = window.calibrationWizard();
      QTest::qWait(30);
      QVERIFY(wizard->grab().save(QString("/tmp/vn_wizard_prepare_%1.png").arg(id)));
      wizard->goToPage(CalibrationWizard::Accelerometer);
      sensor.provideCalibration(resting(1.0));
      wizard->startCapture();
      double t = 1.0;
      int index = 0;
      for (int pose : {0, 1, 2}) {
        const Eigen::Vector3d down = accelerometerPoses()[pose].direction();
        for (const double end = t + 4.5; t < end; t += 0.05) sensor.provideCalibration(resting(t, down, index++));
      }
      const Eigen::Vector3d toward = TiltTarget{"", 38, 95}.direction();
      for (const double end = t + 1.6; t < end; t += 0.05) sensor.provideCalibration(resting(t, toward, index++));
      QTest::qWait(30);
      QVERIFY(wizard->grab().save(QString("/tmp/vn_wizard_accel_%1.png").arg(id)));
      wizard->goToPage(CalibrationWizard::Magnetometer);
      QTest::qWait(30);
      QVERIFY(wizard->grab().save(QString("/tmp/vn_wizard_mag_%1.png").arg(id)));
      wizard->goToPage(CalibrationWizard::Gyro);
      wizard->startCapture();
      for (const double end = t + 31.0; t < end; t += 0.05) sensor.provideCalibration(resting(t, {0, 0, 1}, index++));
      QTest::qWait(150);
      QVERIFY(wizard->grab().save(QString("/tmp/vn_wizard_gyro_%1.png").arg(id)));
      wizard->goToPage(CalibrationWizard::Review);
      QTest::qWait(30);
      QVERIFY(wizard->grab().save(QString("/tmp/vn_wizard_review_%1.png").arg(id)));

      // Directions from a robot config, and the gravity report against its IMU pose.
      QTemporaryDir directory;
      QFile config(directory.filePath("talos.yaml"));
      QVERIFY(config.open(QIODevice::WriteOnly));
      config.write("base_link: [-0.14, 0.03, -0.09]\nimu:\n  pose: [-0.25, -0.1492, 0.0222, 3.141, 0.0, -1.5707]\n");
      config.close();
      QVERIFY(wizard->useRobotConfig(config.fileName()));
      wizard->goToPage(CalibrationWizard::Prepare);
      QTest::qWait(30);
      QVERIFY(wizard->grab().save(QString("/tmp/vn_wizard_robot_prepare_%1.png").arg(id)));
      wizard->goToPage(CalibrationWizard::Accelerometer);
      sensor.provideCalibration(resting(t));
      wizard->startCapture();
      const auto robot_poses = accelerometerPoses(45.0, wizard->guideFrame());
      for (int pose : {0, 1}) {
        for (const double end = t + 4.5; t < end; t += 0.05)
          sensor.provideCalibration(resting(t, robot_poses[pose].direction(), index++));
      }
      QTest::qWait(30);
      QVERIFY(wizard->grab().save(QString("/tmp/vn_wizard_robot_accel_%1.png").arg(id)));
      wizard->goToPage(CalibrationWizard::Gravity);
      wizard->startCapture();
      const Eigen::Vector3d tipped(0.03, -0.02, 1.0);
      for (const double end = t + 12.0; t < end; t += 0.05) sensor.provideCalibration(resting(t, tipped, index++));
      QTest::qWait(150);
      QVERIFY(wizard->grab().save(QString("/tmp/vn_wizard_robot_gravity_%1.png").arg(id)));
      wizard->close();

      // Binary output 1 as checkboxes, with the line load on the command bar.
      for (auto entry = kTalosOutput.cbegin(); entry != kTalosOutput.cend(); ++entry)
        sensor.provideRegister(entry.key(), entry.value());
      auto* tree = window.findChild<QTreeWidget*>("registerTree");
      tree->setCurrentItem(registerItem(tree, 75));
      QTest::qWait(30);
      QVERIFY(window.grab().save(QString("/tmp/vn_dashboard_binary_%1.png").arg(id)));
    }
    applyTheme(qApp, "heat-sheet", QStringLiteral(TEST_SHARE_DIRECTORY));
  }
};

}  // namespace riptide_imu

QTEST_MAIN(riptide_imu::DashboardTest)
#include "test_dashboard.moc"
