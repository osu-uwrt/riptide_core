#include "riptide_imu/config_tool/main_window.hpp"
#include "riptide_imu/config_tool/ros_sensor_transport.hpp"

#include <QAction>
#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QFormLayout>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSettings>
#include <QShowEvent>
#include <QSplitter>
#include <QStatusBar>
#include <QTableWidget>
#include <QTimer>
#include <QToolBar>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <cmath>

namespace riptide_imu {
namespace {

QLabel* title(const QString& text) {
  auto* label = new QLabel(text);
  label->setObjectName("sectionTitle");
  return label;
}

QString vectorText(const Eigen::Vector3d& value, int precision = 7) {
  return QString("[%1, %2, %3]")
      .arg(value.x(), 0, 'g', precision).arg(value.y(), 0, 'g', precision)
      .arg(value.z(), 0, 'g', precision);
}

}  // namespace

MainWindow::MainWindow(QString imu_port, int baud, QString config_file, bool demo,
                       QString share_directory, QWidget* parent,
                       SensorInterface* injected_sensor, QString connection_mode)
    : QMainWindow(parent), share_directory_(std::move(share_directory)),
      connection_mode_(demo ? "demo" : connection_mode.toLower()) {
  qRegisterMetaType<Telemetry>();
  setWindowTitle("VectorNav Workbench");
  setObjectName("vectorNavWorkbench");
  resize(1440, 900);
  setDockOptions(QMainWindow::AnimatedDocks | QMainWindow::AllowNestedDocks |
                 QMainWindow::AllowTabbedDocks | QMainWindow::GroupedDragging);
  setCentralWidget(buildRegistersPage());

  monitor_dock_ = new QDockWidget("Live Sensor", this);
  monitor_dock_->setObjectName("monitorDock");
  monitor_dock_->setWidget(buildMonitorPage());
  addDockWidget(Qt::RightDockWidgetArea, monitor_dock_);

  plot_dock_ = new QDockWidget("Signal Monitor", this);
  plot_dock_->setObjectName("plotDock");
  plot_dock_->setWidget(buildPlotPanel());
  addDockWidget(Qt::BottomDockWidgetArea, plot_dock_);

  calibration_dock_ = new QDockWidget("Calibration", this);
  calibration_dock_->setObjectName("calibrationDock");
  calibration_dock_->setWidget(buildCalibrationPage());
  addDockWidget(Qt::BottomDockWidgetArea, calibration_dock_);

  diagnostics_dock_ = new QDockWidget("Diagnostics & Protocol", this);
  diagnostics_dock_->setObjectName("diagnosticsDock");
  diagnostics_dock_->setWidget(buildDiagnosticsPage());
  addDockWidget(Qt::BottomDockWidgetArea, diagnostics_dock_);
  tabifyDockWidget(plot_dock_, calibration_dock_);
  tabifyDockWidget(calibration_dock_, diagnostics_dock_);
  plot_dock_->raise();

  auto* connection_toolbar = addToolBar("Connection");
  connection_toolbar->setObjectName("connectionToolbar");
  connection_toolbar->setMovable(false);
  connection_toolbar->addWidget(buildConnectionBar(imu_port, baud));
  buildMenusAndToolbar();

  status_ = new QLabel("Disconnected"); statusBar()->addPermanentWidget(status_, 1);
  loadDefinitions(share_directory_ + "/config/registers.json");
  {
    QFile policy_file(share_directory_ + "/config/calibration_policy.json");
    if (policy_file.open(QIODevice::ReadOnly)) {
      const auto object = QJsonDocument::fromJson(policy_file.readAll()).object();
      policy_.minimum_samples = object.value("minimum_samples").toInt(policy_.minimum_samples);
      policy_.gyro_blocks = object.value("gyro_blocks").toInt(policy_.gyro_blocks);
      policy_.gyro_max_rms_rad_s = object.value("gyro_max_rms_rad_s").toDouble(policy_.gyro_max_rms_rad_s);
      policy_.gyro_max_temperature_span_c = object.value("gyro_max_temperature_span_c").toDouble(policy_.gyro_max_temperature_span_c);
      policy_.gyro_max_pose_delta_rad_s = object.value("gyro_max_pose_delta_rad_s").toDouble(policy_.gyro_max_pose_delta_rad_s);
      policy_.accel_max_rms_m_s2 = object.value("accelerometer_max_rms_m_s2").toDouble(policy_.accel_max_rms_m_s2);
      policy_.accel_max_condition = object.value("accelerometer_max_condition").toDouble(policy_.accel_max_condition);
      policy_.accel_min_axis_span_m_s2 = object.value("accelerometer_min_axis_span_m_s2").toDouble(policy_.accel_min_axis_span_m_s2);
      policy_.mag_max_rms_fraction = object.value("magnetometer_max_rms_fraction").toDouble(policy_.mag_max_rms_fraction);
      policy_.mag_min_coverage_deg = object.value("magnetometer_min_coverage_deg").toDouble(policy_.mag_min_coverage_deg);
      policy_.mag_max_tilt_stddev_deg = object.value("magnetometer_max_tilt_stddev_deg").toDouble(policy_.mag_max_tilt_stddev_deg);
    }
  }

  if (injected_sensor) {
    transport_ = injected_sensor;
  } else {
    transport_ = new SensorTransportRouter;
    owns_transport_ = true;
    transport_->moveToThread(&sensor_thread_);
    connect(&sensor_thread_, &QThread::finished, transport_, &QObject::deleteLater);
  }
  connect(this, &MainWindow::requestConnect, transport_, &SensorInterface::connectSensor);
  connect(this, &MainWindow::requestDisconnect, transport_, &SensorInterface::disconnectSensor);
  connect(this, &MainWindow::requestConnectionMode, transport_, &SensorInterface::setConnectionMode);
  connect(this, &MainWindow::requestEndpointDiscovery, transport_, &SensorInterface::discoverEndpoints);
  connect(this, &MainWindow::requestDemo, transport_, &SensorInterface::setDemoMode);
  connect(this, &MainWindow::requestCalibrationCapture, transport_, &SensorInterface::setCalibrationCapture);
  connect(this, &MainWindow::requestOpenMaintenance, transport_, &SensorInterface::openMaintenance);
  connect(this, &MainWindow::requestCloseMaintenance, transport_, &SensorInterface::closeMaintenance);
  connect(this, &MainWindow::requestRead, transport_, &SensorInterface::readRegister);
  connect(this, &MainWindow::requestWrite, transport_, &SensorInterface::writeRegister);
  connect(this, &MainWindow::requestSaveSettings, transport_, &SensorInterface::writeSettings);
  connect(this, &MainWindow::requestTare, transport_, &SensorInterface::tare);
  connect(this, &MainWindow::requestReset, transport_, &SensorInterface::reset);
  connect(transport_, &SensorInterface::telemetryReceived, this, &MainWindow::onTelemetry);
  connect(transport_, &SensorInterface::calibrationSampleReceived,
          this, &MainWindow::onCalibrationSample);
  connect(transport_, &SensorInterface::registerRead, this, &MainWindow::onRegisterRead);
  connect(transport_, &SensorInterface::writeVerified, this, &MainWindow::onRegisterRead);
  connect(transport_, &SensorInterface::connectionChanged, this,
          [this](bool ok, const QString& detail) {
            connected_ = ok;
            if (connection_button_) {
              connection_button_->setEnabled(true);
              connection_button_->setText(ok ? "Disconnect" : "Connect");
            }
            if (port_selector_) port_selector_->setEnabled(!ok);
            if (baud_selector_) baud_selector_->setEnabled(!ok);
            if (connection_mode_selector_) connection_mode_selector_->setEnabled(!ok);
            setStatus(detail, !ok && detail != "Disconnected");
          });
  connect(transport_, &SensorInterface::operationFailed, this,
          [this](const QString& op, const QString& detail) { setStatus(op + ": " + detail, true); });
  connect(transport_, &SensorInterface::operationSucceeded, this,
          [this](const QString& op, const QString& detail) { setStatus(op + ": " + detail); });
  connect(transport_, &SensorInterface::maintenanceChanged, this,
          [this](bool active, const QString& detail) {
            maintenance_active_ = active;
            if (maintenance_button_)
              maintenance_button_->setText(active ? "Leave maintenance" : "Enter maintenance");
            setStatus(detail);
          });
  connect(transport_, &SensorInterface::endpointsDiscovered, this,
          [this, baud](const QStringList& endpoints) {
            if (connection_mode_ != "ros") return;
            const QString selected = port_selector_->currentText();
            port_selector_->clear();
            port_selector_->addItems(endpoints);
            if (!selected.isEmpty()) port_selector_->setCurrentText(selected);
            if (endpoints.size() == 1 && !connected_) {
              port_selector_->setCurrentIndex(0);
              connection_button_->setEnabled(false);
              emit requestConnect(endpoints.front(), baud);
            } else {
              connection_button_->setEnabled(true);
              setStatus(endpoints.isEmpty() ? "No VectorNav drivers discovered" :
                                             "Select a robot endpoint");
            }
          });
  connect(transport_, &SensorInterface::traffic, this,
          [this](const QString& direction, const QString& packet) {
            if (traffic_log_) traffic_log_->appendPlainText(direction + "  " + packet);
          });
  if (owns_transport_) sensor_thread_.start();
  emit requestConnectionMode(connection_mode_ == "ros" ? "ros" : "serial");
  emit requestDemo(demo);
  if (demo || connection_mode_ == "serial" || !imu_port.trimmed().isEmpty()) {
    connection_button_->setEnabled(false);
    setStatus(demo ? "Starting demo data…" : QString("Connecting to %1…").arg(imu_port));
    emit requestConnect(imu_port, baud);
  } else {
    connection_button_->setEnabled(true);
    setStatus("Discovering VectorNav drivers on the ROS network…");
    emit requestEndpointDiscovery();
  }
  if (!config_file.isEmpty()) {
    QString error; configuration_metadata_ = loadControlCenterXml(config_file, &error);
    for (const auto& reg : configuration_metadata_.registers) staged_values_[reg.id] = reg.values;
    refreshRegisterTable();
    if (!error.isEmpty()) setStatus("Configuration import: " + error, true);
  }
  last_sample_clock_.start();
  stale_timer_ = new QTimer(this);
  stale_timer_->setInterval(500);
  connect(stale_timer_, &QTimer::timeout, this, [this] {
    if (telemetry_summary_) telemetry_summary_->markStale(last_sample_clock_.elapsed() > 1200);
  });
  stale_timer_->start();
  restoreDashboardLayout();
}

void MainWindow::buildMenusAndToolbar() {
  auto* file_menu = menuBar()->addMenu("&File");
  auto* import_action = file_menu->addAction("Import Control Center XML…");
  auto* export_action = file_menu->addAction("Export Control Center XML…");
  file_menu->addSeparator();
  auto* export_csv_action = file_menu->addAction("Export telemetry CSV…");
  auto* replay_action = file_menu->addAction("Replay telemetry CSV…");
  file_menu->addSeparator();
  file_menu->addAction("Quit", this, &QWidget::close, QKeySequence::Quit);

  auto* sensor_menu = menuBar()->addMenu("&Sensor");
  auto* read_action = sensor_menu->addAction("Read selected register");
  read_action->setShortcut(QKeySequence::Refresh);
  auto* apply_action = sensor_menu->addAction("Apply staged values");
  apply_action->setShortcut(QKeySequence("Ctrl+Return"));
  auto* discard_action = sensor_menu->addAction("Discard staged values");
  auto* save_action = sensor_menu->addAction("Save active settings to flash…");
  sensor_menu->addSeparator();
  auto* tare_action = sensor_menu->addAction("Tare orientation…");
  auto* reset_action = sensor_menu->addAction("Reset sensor…");

  auto* view_menu = menuBar()->addMenu("&View");
  view_menu->addAction(monitor_dock_->toggleViewAction());
  view_menu->addAction(plot_dock_->toggleViewAction());
  view_menu->addAction(calibration_dock_->toggleViewAction());
  view_menu->addAction(diagnostics_dock_->toggleViewAction());
  view_menu->addSeparator();
  view_menu->addAction("Reset Dashboard Layout", this, &MainWindow::resetDashboardLayout);

  auto* toolbar = addToolBar("Register actions");
  toolbar->setObjectName("registerActionsToolbar");
  toolbar->setMovable(false);
  toolbar->addAction(read_action);
  toolbar->addAction(apply_action);
  toolbar->addAction(discard_action);
  toolbar->addSeparator();
  toolbar->addAction(import_action);
  toolbar->addAction(export_action);
  toolbar->addSeparator();
  toolbar->addAction(save_action);
  toolbar->addSeparator();
  auto* record_action = toolbar->addAction("Record");
  record_action->setCheckable(true);
  toolbar->addAction(replay_action);

  connect(read_action, &QAction::triggered, this, &MainWindow::readSelectedRegister);
  connect(apply_action, &QAction::triggered, this, &MainWindow::applyStagedRegisters);
  connect(discard_action, &QAction::triggered, this, &MainWindow::discardStagedRegisters);
  connect(import_action, &QAction::triggered, this, &MainWindow::importConfiguration);
  connect(export_action, &QAction::triggered, this, &MainWindow::exportConfiguration);
  connect(export_csv_action, &QAction::triggered, this, &MainWindow::exportTelemetry);
  connect(replay_action, &QAction::triggered, this, &MainWindow::loadReplay);
  connect(save_action, &QAction::triggered, this, &MainWindow::savePersistently);
  connect(record_action, &QAction::toggled, this, [this](bool enabled) {
    recording_ = enabled;
    setStatus(enabled ? "Recording telemetry in memory" : "Recording stopped");
  });
  connect(tare_action, &QAction::triggered, this, [this] {
    if (QMessageBox::question(this, "Tare orientation",
                              "Tare changes the sensor's active attitude reference. Continue?") == QMessageBox::Yes)
      emit requestTare();
  });
  connect(reset_action, &QAction::triggered, this, [this] {
    if (QMessageBox::question(this, "Reset sensor",
                              "Reset the sensor? The connection will be interrupted.") == QMessageBox::Yes)
      emit requestReset();
  });
}

void MainWindow::applyDefaultDashboardLayout() {
  monitor_dock_->show();
  plot_dock_->show();
  calibration_dock_->show();
  diagnostics_dock_->show();
  addDockWidget(Qt::RightDockWidgetArea, monitor_dock_);
  addDockWidget(Qt::BottomDockWidgetArea, plot_dock_);
  addDockWidget(Qt::BottomDockWidgetArea, calibration_dock_);
  addDockWidget(Qt::BottomDockWidgetArea, diagnostics_dock_);
  tabifyDockWidget(plot_dock_, calibration_dock_);
  tabifyDockWidget(calibration_dock_, diagnostics_dock_);
  resizeDocks({monitor_dock_}, {360}, Qt::Horizontal);
  resizeDocks({plot_dock_}, {255}, Qt::Vertical);
  plot_dock_->raise();
}

void MainWindow::restoreDashboardLayout() {
  QSettings settings;
  const QByteArray geometry = settings.value("dashboard/geometry").toByteArray();
  const QByteArray state = settings.value("dashboard/state").toByteArray();
  if (!geometry.isEmpty()) restoreGeometry(geometry);
  if (state.isEmpty() || !restoreState(state, 2)) {
    applyDefaultDashboardLayout();
  }
  for (auto* dock : {monitor_dock_, plot_dock_, calibration_dock_, diagnostics_dock_}) {
    const QString key = QString("dashboard/docks/%1/visible").arg(dock->objectName());
    if (settings.contains(key)) {
      const bool visible = settings.value(key).toBool();
      dock->setProperty("restoredVisibility", visible);
      dock->setVisible(visible);
    }
  }
}

void MainWindow::showEvent(QShowEvent* event) {
  QMainWindow::showEvent(event);
  for (auto* dock : {monitor_dock_, plot_dock_, calibration_dock_, diagnostics_dock_}) {
    const QVariant restored = dock->property("restoredVisibility");
    if (restored.isValid()) {
      const bool visible = restored.toBool();
      QTimer::singleShot(10, dock, [dock, visible] {
        dock->setVisible(visible);
        dock->setProperty("restoredVisibility", QVariant());
      });
    }
  }
}

void MainWindow::resetDashboardLayout() {
  QSettings settings;
  settings.remove("dashboard/geometry");
  settings.remove("dashboard/state");
  settings.remove("dashboard/docks");
  resize(1440, 900);
  applyDefaultDashboardLayout();
  setStatus("Dashboard layout reset");
}

QWidget* MainWindow::buildConnectionBar(const QString& initial_port, int initial_baud) {
  auto* bar = new QWidget;
  bar->setObjectName("connectionBar");
  auto* layout = new QHBoxLayout(bar);
  layout->setContentsMargins(10, 6, 10, 6);

  connection_mode_selector_ = new QComboBox;
  connection_mode_selector_->setObjectName("connectionModeSelector");
  connection_mode_selector_->addItem("Robot over ROS", "ros");
  connection_mode_selector_->addItem("Local serial", "serial");
  connection_mode_selector_->addItem("Demo", "demo");
  const int initial_mode = connection_mode_selector_->findData(connection_mode_);
  connection_mode_selector_->setCurrentIndex(initial_mode >= 0 ? initial_mode : 0);
  layout->addWidget(connection_mode_selector_);

  connection_target_label_ = new QLabel(connection_mode_ == "ros" ? "Robot" : "Sensor port");
  layout->addWidget(connection_target_label_);

  port_selector_ = new QComboBox;
  port_selector_->setObjectName("sensorPortSelector");
  port_selector_->setAccessibleName("Sensor serial port");
  port_selector_->setEditable(true);
  port_selector_->setInsertPolicy(QComboBox::NoInsert);
  port_selector_->setMinimumContentsLength(24);
  port_selector_->setToolTip("Select a discovered robot or enter a local serial device path.");
  if (!initial_port.trimmed().isEmpty()) port_selector_->addItem(initial_port.trimmed());
  if (connection_mode_ != "ros") refreshSerialPorts();
  port_selector_->setCurrentText(initial_port);
  layout->addWidget(port_selector_, 1);

  auto* refresh = new QPushButton("Refresh");
  refresh->setObjectName("refreshSerialPortsButton");
  refresh->setToolTip("Rediscover ROS drivers or rescan Linux serial devices");
  connect(refresh, &QPushButton::clicked, this, &MainWindow::refreshConnectionTargets);
  layout->addWidget(refresh);

  baud_label_ = new QLabel("Baud");
  layout->addWidget(baud_label_);
  baud_selector_ = new QComboBox;
  baud_selector_->setObjectName("sensorBaudSelector");
  baud_selector_->setAccessibleName("Sensor baud rate");
  baud_selector_->setEditable(true);
  baud_selector_->addItems({"9600", "19200", "38400", "57600", "115200", "128000",
                            "230400", "460800", "921600"});
  baud_selector_->setCurrentText(QString::number(initial_baud));
  layout->addWidget(baud_selector_);

  maintenance_button_ = new QPushButton("Enter maintenance");
  maintenance_button_->setObjectName("maintenanceButton");
  maintenance_button_->setToolTip("Required for settings that can interrupt the live driver");
  connect(maintenance_button_, &QPushButton::clicked, this, [this] {
    if (maintenance_active_) {
      emit requestCloseMaintenance();
      return;
    }
    if (QMessageBox::warning(
            this, "Enter sensor maintenance",
            "Driver-critical writes, reset, and persistent save can briefly interrupt IMU data. "
            "The driver process will remain running. Enter maintenance mode?",
            QMessageBox::Yes | QMessageBox::Cancel) == QMessageBox::Yes)
      emit requestOpenMaintenance();
  });
  layout->addWidget(maintenance_button_);

  connection_button_ = new QPushButton("Connect");
  connection_button_->setAccessibleName("Connect or disconnect sensor");
  connection_button_->setObjectName("primaryButton");
  connect(connection_button_, &QPushButton::clicked, this, &MainWindow::toggleConnection);
  layout->addWidget(connection_button_);
  connect(connection_mode_selector_, qOverload<int>(&QComboBox::currentIndexChanged),
          this, [this](int index) {
    connection_mode_ = connection_mode_selector_->itemData(index).toString();
    maintenance_active_ = false;
    maintenance_button_->setText("Enter maintenance");
    connection_target_label_->setText(connection_mode_ == "ros" ? "Robot" : "Sensor port");
    baud_selector_->setVisible(connection_mode_ != "ros");
    baud_label_->setVisible(connection_mode_ != "ros");
    emit requestConnectionMode(connection_mode_ == "ros" ? "ros" : "serial");
    emit requestDemo(connection_mode_ == "demo");
    port_selector_->clear();
    refreshConnectionTargets();
    connection_button_->setText("Connect");
    connection_button_->setEnabled(true);
  });
  baud_selector_->setVisible(connection_mode_ != "ros");
  baud_label_->setVisible(connection_mode_ != "ros");
  return bar;
}

void MainWindow::refreshConnectionTargets() {
  if (connection_mode_ == "ros") {
    emit requestEndpointDiscovery();
  } else {
    refreshSerialPorts();
  }
}

void MainWindow::refreshSerialPorts() {
  if (!port_selector_) return;
  const QString selected = port_selector_->currentText().trimmed();
  QStringList ports;
  if (!selected.isEmpty()) ports << selected;

  const QDir stable_directory("/dev/serial/by-id");
  for (const QString& entry : stable_directory.entryList(QDir::Files | QDir::System | QDir::NoDotAndDotDot,
                                                          QDir::Name)) {
    ports << stable_directory.absoluteFilePath(entry);
  }
  const QDir devices("/dev");
  for (const QString& pattern : {QString("ttyUSB*"), QString("ttyACM*"), QString("ttyTHS*")}) {
    for (const QString& entry : devices.entryList({pattern}, QDir::Files | QDir::System, QDir::Name))
      ports << devices.absoluteFilePath(entry);
  }
  ports.removeDuplicates();

  const QSignalBlocker blocker(port_selector_);
  port_selector_->clear();
  port_selector_->addItems(ports);
  port_selector_->setCurrentText(selected);
}

void MainWindow::toggleConnection() {
  if (connected_) {
    connection_button_->setEnabled(false);
    setStatus("Disconnecting…");
    emit requestDisconnect();
    return;
  }

  QString port = port_selector_->currentText().trimmed();
  if (connection_mode_ == "ros" && port.isEmpty()) port = "/";
  if (connection_mode_ == "demo") port = "demo";
  bool valid_baud = false;
  const int baud = baud_selector_->currentText().trimmed().toInt(&valid_baud);
  if (port.isEmpty()) {
    setStatus("Select or enter a sensor port", true);
    port_selector_->setFocus();
    return;
  }
  if (connection_mode_ != "ros" && (!valid_baud || baud <= 0)) {
    setStatus("Enter a valid positive baud rate", true);
    baud_selector_->setFocus();
    return;
  }
  connection_button_->setEnabled(false);
  setStatus(QString("Connecting to %1…").arg(port));
  emit requestConnect(port, baud);
}

MainWindow::~MainWindow() {
  QSettings settings;
  settings.setValue("dashboard/geometry", saveGeometry());
  settings.setValue("dashboard/state", saveState(2));
  for (auto* dock : {monitor_dock_, plot_dock_, calibration_dock_, diagnostics_dock_}) {
    settings.setValue(QString("dashboard/docks/%1/visible").arg(dock->objectName()),
                      !dock->isHidden());
  }
  settings.sync();
  if (owns_transport_) {
    QMetaObject::invokeMethod(transport_, "disconnectSensor", Qt::BlockingQueuedConnection);
    sensor_thread_.quit();
    sensor_thread_.wait(3000);
  }
}

QWidget* MainWindow::buildMonitorPage() {
  auto* page = new QWidget;
  auto* layout = new QVBoxLayout(page);
  layout->setContentsMargins(6, 6, 6, 6);
  layout->setSpacing(5);
  attitude_ = new AttitudeWidget;
  telemetry_summary_ = new TelemetrySummaryWidget;
  auto* splitter = new QSplitter(Qt::Vertical);
  splitter->setObjectName("monitorSplitter");
  splitter->addWidget(attitude_);
  splitter->addWidget(telemetry_summary_);
  splitter->setStretchFactor(0, 1);
  splitter->setStretchFactor(1, 1);
  layout->addWidget(splitter, 1);
  return page;
}

QWidget* MainWindow::buildPlotPanel() {
  auto* page = new QWidget;
  auto* layout = new QVBoxLayout(page);
  layout->setContentsMargins(6, 6, 6, 6);
  auto* controls = new QHBoxLayout;
  auto* pause = new QPushButton("Pause display");
  auto* export_button = new QPushButton("Export CSV…");
  auto* replay = new QPushButton("Replay CSV…");
  controls->addWidget(new QLabel("Angular rate · X / Y / Z"));
  controls->addStretch();
  controls->addWidget(pause);
  controls->addWidget(export_button);
  controls->addWidget(replay);
  layout->addLayout(controls);
  connect(pause, &QPushButton::clicked, this, [this, pause] {
    monitor_paused_ = !monitor_paused_;
    pause->setText(monitor_paused_ ? "Resume display" : "Pause display");
  });
  connect(export_button, &QPushButton::clicked, this, &MainWindow::exportTelemetry);
  connect(replay, &QPushButton::clicked, this, &MainWindow::loadReplay);
  plot_ = new PlotWidget;
  layout->addWidget(plot_, 1);
  return page;
}

QWidget* MainWindow::buildRegistersPage() {
  auto* page = new QWidget;
  page->setObjectName("registerWorkspace");
  auto* layout = new QVBoxLayout(page);
  layout->setContentsMargins(8, 8, 8, 8);
  layout->setSpacing(6);

  auto* heading = new QHBoxLayout;
  heading->addWidget(title("Register Workbench"));
  register_state_ = new QLabel("NO REGISTER SELECTED");
  register_state_->setObjectName("stagedBadge");
  heading->addStretch();
  heading->addWidget(register_state_);
  layout->addLayout(heading);

  register_filter_ = new QLineEdit;
  register_filter_->setObjectName("registerSearch");
  register_filter_->setClearButtonEnabled(true);
  register_filter_->setPlaceholderText("Filter register ID, group, name, or description…  (Ctrl+F)");
  layout->addWidget(register_filter_);

  auto* splitter = new QSplitter(Qt::Horizontal);
  splitter->setObjectName("registerSplitter");
  register_tree_ = new QTreeWidget;
  register_tree_->setObjectName("registerTree");
  register_tree_->setColumnCount(2);
  register_tree_->setHeaderLabels({"Register", "State"});
  register_tree_->setAlternatingRowColors(true);
  register_tree_->setMinimumWidth(320);
  register_tree_->header()->setSectionResizeMode(0, QHeaderView::Stretch);
  register_tree_->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  splitter->addWidget(register_tree_);

  auto* inspector = new QWidget;
  auto* inspector_layout = new QVBoxLayout(inspector);
  inspector_layout->setContentsMargins(8, 0, 0, 0);
  register_description_ = new QLabel("Select a register to inspect its current and staged values.");
  register_description_->setWordWrap(true);
  register_description_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  inspector_layout->addWidget(register_description_);

  register_inspector_ = new QTableWidget(0, 4);
  register_inspector_->setObjectName("registerInspector");
  register_inspector_->setHorizontalHeaderLabels({"Field", "Device", "Staged", "Units"});
  register_inspector_->setAlternatingRowColors(true);
  register_inspector_->verticalHeader()->setVisible(false);
  register_inspector_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  register_inspector_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
  register_inspector_->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch);
  register_inspector_->horizontalHeader()->setSectionResizeMode(3, QHeaderView::ResizeToContents);
  inspector_layout->addWidget(register_inspector_, 1);

  auto* raw_row = new QHBoxLayout;
  raw_row->addWidget(new QLabel("Raw protocol values"));
  raw_register_editor_ = new QLineEdit;
  raw_register_editor_->setObjectName("rawRegisterEditor");
  raw_register_editor_->setPlaceholderText("Semicolon-separated register fields");
  raw_row->addWidget(raw_register_editor_, 1);
  auto* stage_raw = new QPushButton("Stage raw");
  raw_row->addWidget(stage_raw);
  inspector_layout->addLayout(raw_row);
  splitter->addWidget(inspector);
  splitter->setStretchFactor(0, 2);
  splitter->setStretchFactor(1, 5);
  layout->addWidget(splitter, 1);

  connect(register_filter_, &QLineEdit::textChanged, this, [this] { refreshRegisterTable(); });
  connect(register_tree_, &QTreeWidget::currentItemChanged, this, &MainWindow::selectRegister);
  connect(register_inspector_, &QTableWidget::cellChanged, this, [this](int, int column) {
    if (column == 2) stageInspectorValues();
  });
  connect(stage_raw, &QPushButton::clicked, this, [this] {
    if (current_register_id_ < 0) return;
    const auto definition = definitionForId(current_register_id_);
    QString error;
    if (!validateRegisterValue(definition, raw_register_editor_->text(), &error)) {
      setStatus(definition.name + ": " + error, true);
      return;
    }
    staged_values_[current_register_id_] = raw_register_editor_->text();
    refreshRegisterTable();
    setStatus(definition.name + " staged; sensor unchanged");
  });
  auto* search_shortcut = new QAction(this);
  search_shortcut->setShortcut(QKeySequence::Find);
  addAction(search_shortcut);
  connect(search_shortcut, &QAction::triggered, register_filter_, qOverload<>(&QWidget::setFocus));
  return page;
}

QWidget* MainWindow::buildCalibrationPage() {
  auto* page = new QWidget; auto* layout = new QVBoxLayout(page); layout->addWidget(title("Guided calibration"));
  auto* explanation = new QLabel(
      "Steps: 1 Gyro stationary poses · 2 Accelerometer varied stationary poses · 3 Magnetometer horizontal circle · "
      "4 Gravity report. Each step is independent. Skipped steps preserve their registers. Collection never writes the sensor.");
  explanation->setWordWrap(true); layout->addWidget(explanation);
  auto* row = new QHBoxLayout; auto* selector = new QComboBox;
  selector->setObjectName("calibrationStepSelector");
  selector->addItems({"Gyro bias", "Accelerometer bias + magnitude", "Planar magnetometer", "Gravity/tilt report"});
  calibration_sample_count_ = new QLabel("0 samples");
  calibration_sample_count_->setObjectName("calibrationSampleCount");
  connect(selector, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int index) {
    if (capturing_) toggleCapture();
    calibration_step_ = index;
    const std::size_t count = index == 0 ? gyro_samples_.size() :
        index == 1 ? accel_samples_.size() : index == 2 ? mag_samples_.size() : gravity_samples_.size();
    calibration_sample_count_->setText(QString("%1 / %2 minimum samples").arg(count).arg(policy_.minimum_samples));
  });
  capture_button_ = new QPushButton("Start capture"); auto* evaluate = new QPushButton("Evaluate quality");
  capture_button_->setObjectName("calibrationCaptureButton");
  auto* stage = new QPushButton("Stage passed result"); auto* clear = new QPushButton("Clear selected samples");
  row->addWidget(selector); row->addWidget(capture_button_); row->addWidget(evaluate); row->addWidget(stage); row->addWidget(clear); row->addWidget(calibration_sample_count_); layout->addLayout(row);
  calibration_result_ = new QPlainTextEdit; calibration_result_->setReadOnly(true); layout->addWidget(calibration_result_, 1);
  auto* export_button = new QPushButton("Export machine-readable report…"); layout->addWidget(export_button);
  connect(capture_button_, &QPushButton::clicked, this, &MainWindow::toggleCapture);
  connect(evaluate, &QPushButton::clicked, this, &MainWindow::evaluateCalibration);
  connect(stage, &QPushButton::clicked, this, &MainWindow::stageCalibrationResult);
  connect(export_button, &QPushButton::clicked, this, &MainWindow::exportCalibrationReport);
  connect(clear, &QPushButton::clicked, this, [this] {
    if (calibration_step_ == 0) gyro_samples_.clear(); else if (calibration_step_ == 1) accel_samples_.clear();
    else if (calibration_step_ == 2) { mag_samples_.clear(); mag_accel_samples_.clear(); }
    else gravity_samples_.clear();
    calibration_sample_count_->setText(QString("0 / %1 minimum samples").arg(policy_.minimum_samples));
    calibration_result_->appendPlainText("Selected samples cleared; sensor registers unchanged.");
  });
  return page;
}

QWidget* MainWindow::buildDiagnosticsPage() {
  auto* page = new QWidget; auto* layout = new QVBoxLayout(page); layout->addWidget(title("Diagnostics"));
  traffic_log_ = new QPlainTextEdit; traffic_log_->setReadOnly(true); traffic_log_->document()->setMaximumBlockCount(2000); layout->addWidget(traffic_log_, 1);
  auto* row = new QHBoxLayout; auto* tare_button = new QPushButton("Tare orientation…"); auto* reset_button = new QPushButton("Reset sensor…"); auto* clear = new QPushButton("Clear log");
  row->addWidget(tare_button); row->addWidget(reset_button); row->addStretch(); row->addWidget(clear); layout->addLayout(row);
  connect(clear, &QPushButton::clicked, traffic_log_, &QPlainTextEdit::clear);
  connect(tare_button, &QPushButton::clicked, this, [this] {
    if (QMessageBox::question(this, "Tare orientation", "Tare changes the sensor's active attitude reference. Continue?") == QMessageBox::Yes) emit requestTare();
  });
  connect(reset_button, &QPushButton::clicked, this, [this] {
    if (QMessageBox::question(this, "Reset sensor", "Reset the sensor? The connection will be interrupted.") == QMessageBox::Yes) emit requestReset();
  }); return page;
}

void MainWindow::loadDefinitions(const QString& path) {
  QString error; definitions_ = loadRegisterDefinitions(path, &error);
  if (!error.isEmpty()) setStatus("Register schema: " + error, true);
  refreshRegisterTable();
}

void MainWindow::refreshRegisterTable() {
  if (!register_tree_) return;
  const QSignalBlocker blocker(register_tree_);
  const int selected_id = current_register_id_;
  register_tree_->clear();
  const QString filter = register_filter_ ? register_filter_->text().trimmed() : QString();
  QMap<QString, QTreeWidgetItem*> groups;
  QTreeWidgetItem* selected_item = nullptr;
  for (const auto& definition : definitions_) {
    const QString searchable = QString("%1 %2 %3 %4").arg(definition.id).arg(definition.group, definition.name, definition.description);
    if (!filter.isEmpty() && !searchable.contains(filter, Qt::CaseInsensitive)) continue;
    auto* group = groups.value(definition.group, nullptr);
    if (!group) {
      group = new QTreeWidgetItem(register_tree_, {definition.group});
      group->setFirstColumnSpanned(true);
      group->setFlags(group->flags() & ~Qt::ItemIsSelectable);
      QFont group_font = group->font(0);
      group_font.setBold(true);
      group->setFont(0, group_font);
      groups.insert(definition.group, group);
    }
    QString state = definition.writable ? "" : "READ";
    if (device_values_.contains(definition.id)) state = "DEVICE";
    if (staged_values_.contains(definition.id)) state = "STAGED";
    auto* item = new QTreeWidgetItem(group,
        {QString("%1  %2").arg(definition.id, 3).arg(definition.name), state});
    item->setData(0, Qt::UserRole, definition.id);
    item->setToolTip(0, definition.description);
    if (staged_values_.contains(definition.id)) {
      item->setForeground(0, QColor("#7dd3fc"));
      item->setForeground(1, QColor("#7dd3fc"));
    } else if (!definition.writable) {
      item->setForeground(1, QColor("#7f8b99"));
    }
    if (definition.id == selected_id) selected_item = item;
  }
  register_tree_->expandAll();
  if (!selected_item && register_tree_->topLevelItemCount() > 0 &&
      register_tree_->topLevelItem(0)->childCount() > 0) {
    selected_item = register_tree_->topLevelItem(0)->child(0);
    current_register_id_ = selected_item->data(0, Qt::UserRole).toInt();
  }
  if (selected_item) register_tree_->setCurrentItem(selected_item);
  refreshRegisterInspector();
}

QStringList MainWindow::fieldNames(const RegisterDefinition& definition, int count) const {
  QStringList names;
  for (const auto& field : definition.fields) names << field.name;
  if (!names.isEmpty()) return names;
  if ((definition.id == 23 || definition.id == 25 || definition.id == 84) && count == 12)
    return {"C11", "C12", "C13", "C21", "C22", "C23", "C31", "C32", "C33", "Bias X", "Bias Y", "Bias Z"};
  if (count == 9 && definition.name.contains("Rotation"))
    return {"R11", "R12", "R13", "R21", "R22", "R23", "R31", "R32", "R33"};
  if (count == 4 && definition.name.contains("Quaternion", Qt::CaseInsensitive))
    return {"X", "Y", "Z", "W"};
  if (count == 3) return {"X", "Y", "Z"};
  for (int index = 0; index < count; ++index) names << QString("Field %1").arg(index + 1);
  return names;
}

void MainWindow::selectRegister(QTreeWidgetItem* current, QTreeWidgetItem*) {
  if (!current || !current->parent()) return;
  current_register_id_ = current->data(0, Qt::UserRole).toInt();
  refreshRegisterInspector();
}

void MainWindow::refreshRegisterInspector() {
  if (!register_inspector_ || !raw_register_editor_) return;
  const RegisterDefinition definition = definitionForId(current_register_id_);
  updating_register_editor_ = true;
  register_inspector_->setRowCount(0);
  if (definition.id < 0) {
    register_description_->setText("Select a register to inspect its current and staged values.");
    register_state_->setText("NO REGISTER SELECTED");
    raw_register_editor_->clear();
    raw_register_editor_->setEnabled(false);
    updating_register_editor_ = false;
    return;
  }

  const QString device_text = device_values_.value(definition.id);
  const QString staged_text = staged_values_.value(definition.id, device_text);
  const QStringList device = device_text.split(';', Qt::KeepEmptyParts);
  const QStringList staged = staged_text.split(';', Qt::KeepEmptyParts);
  int count = std::max(device.size(), staged.size());
  count = std::max(count, definition.fields.size());
  if (definition.value_count >= 0) count = std::max(count, definition.value_count);
  if (device_text.isEmpty() && staged_text.isEmpty() && definition.value_count < 0) count = 1;
  const QStringList names = fieldNames(definition, count);

  register_description_->setText(QString("%1 · Register %2 · %3%4")
      .arg(definition.group).arg(definition.id).arg(definition.description)
      .arg(definition.firmware.isEmpty() ? QString() : " · Firmware " + definition.firmware));
  register_state_->setText(staged_values_.contains(definition.id) ? "STAGED — SENSOR UNCHANGED" :
                           device_values_.contains(definition.id) ? "DEVICE VALUE" : "NOT READ");
  register_inspector_->setRowCount(count);
  for (int row = 0; row < count; ++row) {
    QString field_name = names.value(row, QString("Field %1").arg(row + 1));
    QString units = definition.units;
    if (row < definition.fields.size()) {
      const auto& field = definition.fields[row];
      if (!field.units.isEmpty()) units = field.units;
      if (!field.enum_values.isEmpty()) {
        QStringList choices;
        for (auto value = field.enum_values.cbegin(); value != field.enum_values.cend(); ++value)
          choices << value.key() + "=" + value.value();
        field_name += "  [" + choices.join(", ") + "]";
      }
    }
    auto* name_item = new QTableWidgetItem(field_name);
    auto* device_item = new QTableWidgetItem(device.value(row));
    auto* staged_item = new QTableWidgetItem(staged.value(row));
    auto* units_item = new QTableWidgetItem(units);
    name_item->setFlags(name_item->flags() & ~Qt::ItemIsEditable);
    device_item->setFlags(device_item->flags() & ~Qt::ItemIsEditable);
    units_item->setFlags(units_item->flags() & ~Qt::ItemIsEditable);
    if (!definition.writable) staged_item->setFlags(staged_item->flags() & ~Qt::ItemIsEditable);
    if (device.value(row) != staged.value(row)) staged_item->setBackground(QColor("#173b58"));
    register_inspector_->setItem(row, 0, name_item);
    register_inspector_->setItem(row, 1, device_item);
    register_inspector_->setItem(row, 2, staged_item);
    register_inspector_->setItem(row, 3, units_item);
    if (definition.writable && row < definition.fields.size()) {
      const auto& field = definition.fields[row];
      if (!field.enum_values.isEmpty()) {
        auto* editor = new QComboBox;
        for (auto value = field.enum_values.cbegin(); value != field.enum_values.cend(); ++value)
          editor->addItem(value.value() + "  [" + value.key() + "]", value.key());
        const int selected = editor->findData(staged.value(row));
        if (selected >= 0) editor->setCurrentIndex(selected);
        register_inspector_->setCellWidget(row, 2, editor);
        connect(editor, qOverload<int>(&QComboBox::currentIndexChanged), this,
                [this](int) { stageInspectorValues(); });
      } else if (field.has_range) {
        auto* editor = new QDoubleSpinBox;
        editor->setRange(field.minimum, field.maximum);
        editor->setDecimals(7);
        editor->setValue(staged.value(row).toDouble());
        register_inspector_->setCellWidget(row, 2, editor);
        connect(editor, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
                [this](double) { stageInspectorValues(); });
      }
    }
  }
  raw_register_editor_->setText(staged_text);
  raw_register_editor_->setEnabled(definition.writable);
  updating_register_editor_ = false;
}

void MainWindow::stageInspectorValues() {
  if (updating_register_editor_ || current_register_id_ < 0) return;
  const auto definition = definitionForId(current_register_id_);
  if (!definition.writable) return;
  QStringList values;
  for (int row = 0; row < register_inspector_->rowCount(); ++row) {
    if (auto* choice = qobject_cast<QComboBox*>(register_inspector_->cellWidget(row, 2)))
      values << choice->currentData().toString();
    else if (auto* number = qobject_cast<QDoubleSpinBox*>(register_inspector_->cellWidget(row, 2)))
      values << QString::number(number->value(), 'g', 12);
    else
      values << register_inspector_->item(row, 2)->text().trimmed();
  }
  const QString serialized = values.join(';');
  QString error;
  if (!validateRegisterValue(definition, serialized, &error)) {
    setStatus(definition.name + ": " + error, true);
    return;
  }
  if (serialized == device_values_.value(current_register_id_))
    staged_values_.remove(current_register_id_);
  else
    staged_values_[current_register_id_] = serialized;
  raw_register_editor_->setText(serialized);
  register_state_->setText("STAGED — SENSOR UNCHANGED");
  setStatus(definition.name + " staged; sensor unchanged");
}

void MainWindow::setStatus(const QString& text, bool error) {
  if (!status_) return;
  status_->setText(text);
  status_->setStyleSheet(error ? "color:#ff6262" : "");
}

RegisterDefinition MainWindow::definitionForId(int id) const {
  for (const auto& definition : definitions_) if (definition.id == id) return definition;
  return {};
}

void MainWindow::onTelemetry(const Telemetry& sample) {
  last_sample_clock_.restart();
  if (recording_) {
    recorded_telemetry_.push_back(sample);
    if (recorded_telemetry_.size() > 500000)
      recorded_telemetry_.erase(recorded_telemetry_.begin(), recorded_telemetry_.begin() + 10000);
  }
  if (monitor_paused_) return;
  attitude_->setQuaternion(sample.qx, sample.qy, sample.qz, sample.qw);
  plot_->append(sample);
  telemetry_summary_->updateSample(sample);
}

void MainWindow::onCalibrationSample(const Telemetry& sample) {
  if (!capturing_) return;
  const double t = capture_clock_.elapsed() / 1000.0;
  if (calibration_step_ == 0)
    gyro_samples_.push_back({t, {sample.gx, sample.gy, sample.gz}, sample.temperature_c});
  else if (calibration_step_ == 1)
    accel_samples_.push_back({t, {sample.ax, sample.ay, sample.az}, sample.temperature_c});
  else if (calibration_step_ == 3)
    gravity_samples_.push_back({t, {sample.ax, sample.ay, sample.az}, sample.temperature_c});
  else {
    mag_samples_.push_back({t, {sample.mx, sample.my, sample.mz}, sample.temperature_c});
    mag_accel_samples_.push_back({t, {sample.ax, sample.ay, sample.az}, sample.temperature_c});
  }
  const std::size_t count = calibration_step_ == 0 ? gyro_samples_.size() :
      calibration_step_ == 1 ? accel_samples_.size() :
      calibration_step_ == 2 ? mag_samples_.size() : gravity_samples_.size();
  calibration_sample_count_->setText(QString("%1 / %2 minimum samples").arg(count).arg(policy_.minimum_samples));
}

void MainWindow::onRegisterRead(int id, const QString& values) {
  device_values_[id] = values; if (staged_values_.value(id) == values) staged_values_.remove(id); refreshRegisterTable();
}

void MainWindow::readSelectedRegister() {
  if (current_register_id_ < 0) {
    setStatus("Select a register to read", true);
    return;
  }
  emit requestRead(current_register_id_);
}

void MainWindow::applyStagedRegisters() {
  if (staged_values_.isEmpty()) { setStatus("No staged changes"); return; }
  const QMap<int, QString> pending = staged_values_;
  for (auto iterator = pending.cbegin(); iterator != pending.cend(); ++iterator) {
    QString error; const auto definition = definitionForId(iterator.key());
    if (definition.id < 0) { setStatus(QString("Register %1 is unsupported by this schema").arg(iterator.key()), true); return; }
    if (!validateRegisterValue(definition, iterator.value(), &error)) { setStatus(definition.name + ": " + error, true); return; }
    if (connection_mode_ == "ros" && definition.driver_critical && !maintenance_active_) {
      setStatus(definition.name + " requires an active maintenance session", true);
      return;
    }
    if (definition.dangerous && QMessageBox::question(this, "Connection-affecting register", definition.name + " can interrupt the connection. Apply it?") != QMessageBox::Yes) return;
  }
  for (auto iterator = pending.cbegin(); iterator != pending.cend(); ++iterator)
    emit requestWrite(iterator.key(), iterator.value());
}

void MainWindow::discardStagedRegisters() { staged_values_.clear(); refreshRegisterTable(); setStatus("Staged changes discarded"); }

void MainWindow::importConfiguration() {
  const QString path = QFileDialog::getOpenFileName(this, "Import Control Center configuration", {}, "VectorNav XML (*.xml)"); if (path.isEmpty()) return;
  QString error; const auto imported = loadControlCenterXml(path, &error); if (!error.isEmpty()) { setStatus(error, true); return; }
  if (QMessageBox::question(this, "Stage configuration", QString("Stage %1 register entries? Nothing will be written yet.").arg(imported.registers.size())) != QMessageBox::Yes) return;
  configuration_metadata_ = imported; for (const auto& reg : imported.registers) staged_values_[reg.id] = reg.values; refreshRegisterTable();
}

void MainWindow::exportConfiguration() {
  const QString path = QFileDialog::getSaveFileName(this, "Export Control Center configuration", {}, "VectorNav XML (*.xml)"); if (path.isEmpty()) return;
  SensorConfiguration output = configuration_metadata_; output.registers.clear();
  for (const auto& definition : definitions_) if (device_values_.contains(definition.id) || staged_values_.contains(definition.id))
    output.registers.push_back({definition.id, definition.name, staged_values_.value(definition.id, device_values_.value(definition.id)), true});
  for (const auto& unknown : configuration_metadata_.registers) if (definitionForId(unknown.id).id < 0) output.registers.push_back(unknown);
  QString error; if (!saveControlCenterXml(path, output, &error)) setStatus(error, true); else setStatus("Configuration exported");
}

void MainWindow::savePersistently() {
  if (!staged_values_.isEmpty()) { QMessageBox::warning(this, "Unsaved staged values", "Apply and verify staged changes before saving persistent sensor state."); return; }
  if (connection_mode_ == "ros" && !maintenance_active_) {
    setStatus("Enter maintenance mode before persistent save", true);
    return;
  }
  if (QMessageBox::question(
          this, "Save, reset, and verify",
          "Persist the complete active configuration, reset the sensor, reconnect the driver, "
          "and verify all values read by this dashboard? IMU output will be stale briefly.") !=
      QMessageBox::Yes) return;
  QJsonObject expected;
  for (const auto& definition : definitions_) {
    if (!definition.writable || !definition.persistent ||
        !device_values_.contains(definition.id)) continue;
    QJsonArray fields;
    for (const auto& value : device_values_.value(definition.id).split(';')) fields.push_back(value);
    expected[QString::number(definition.id)] = fields;
  }
  emit requestSaveSettings(QString::fromUtf8(QJsonDocument(expected).toJson(QJsonDocument::Compact)));
}

void MainWindow::toggleCapture() {
  capturing_ = !capturing_; if (capturing_) capture_clock_.restart();
  emit requestCalibrationCapture(capturing_);
  capture_button_->setText(capturing_ ? "Stop capture" : "Start capture");
  calibration_result_->appendPlainText(capturing_ ? "Capture started. Keep each requested pose stationary; stop between motions." : "Capture stopped; no sensor values changed.");
}

void MainWindow::evaluateCalibration() {
  if (capturing_) toggleCapture();
  if (calibration_step_ == 0) { gyro_result_ = estimateGyroBias(gyro_samples_, policy_); calibration_result_->appendPlainText(QString::fromStdString(gyro_result_.quality.summary) + "\nResidual " + vectorText(gyro_result_.residual_bias)); }
  else if (calibration_step_ == 1) { accel_result_ = estimateAccelerometerBias(accel_samples_, policy_); calibration_result_->appendPlainText(QString::fromStdString(accel_result_.quality.summary) + "\nBias " + vectorText(accel_result_.bias) + QString("; measured magnitude %1 ± %2 m/s²").arg(accel_result_.measured_gravity).arg(accel_result_.gravity_stddev)); }
  else if (calibration_step_ == 2) { mag_result_ = estimatePlanarMagBias(mag_samples_, mag_accel_samples_, policy_); calibration_result_->appendPlainText(QString::fromStdString(mag_result_.quality.summary) + "\nObservable in-plane bias " + vectorText(mag_result_.in_plane_bias)); }
  else { gravity_report_ = makeGravityReport(gravity_samples_, policy_); calibration_result_->appendPlainText(QString::fromStdString(gravity_report_.quality.summary) + "\nSpecific force " + vectorText(gravity_report_.specific_force_mean) + "\nGravity vector (opposite specific force) " + vectorText(gravity_report_.gravity_vector) + QString("\nMagnitude %1 ± %2 m/s²; tilt from sensor +Z %3 ± %4 deg. This is not sensor-to-robot mounting rotation.").arg(gravity_report_.magnitude).arg(gravity_report_.magnitude_stddev).arg(gravity_report_.tilt_from_sensor_z_deg).arg(gravity_report_.tilt_stddev_deg)); }
}

bool MainWindow::parseCompensation(const QString& text, Eigen::Matrix3d* matrix, Eigen::Vector3d* bias) {
  const auto fields = text.split(';'); if (fields.size() != 12) return false;
  bool ok = true; for (int i = 0; i < 9; ++i) { bool part=false; (*matrix)(i/3,i%3)=fields[i].toDouble(&part); ok &= part; }
  for (int i = 0; i < 3; ++i) { bool part=false; (*bias)(i)=fields[i+9].toDouble(&part); ok &= part; } return ok;
}

QString MainWindow::formatCompensation(const Eigen::Matrix3d& matrix, const Eigen::Vector3d& bias) {
  QStringList fields; for (int row=0; row<3; ++row) for(int column=0;column<3;++column) fields << QString::number(static_cast<float>(matrix(row,column)),'g',9);
  for(int i=0;i<3;++i) fields << QString::number(static_cast<float>(bias(i)),'g',9);
  return fields.join(';');
}

void MainWindow::stageCalibrationResult() {
  const int id = calibration_step_ == 0 ? 84 : calibration_step_ == 1 ? 25 : calibration_step_ == 2 ? 23 : -1;
  if (id < 0) { setStatus("Gravity report has no register update by design"); return; }
  const Quality quality = calibration_step_ == 0 ? gyro_result_.quality : calibration_step_ == 1 ? accel_result_.quality : mag_result_.quality;
  if (!quality.passed) { setStatus("Quality gate failed; result cannot be staged", true); return; }
  Eigen::Matrix3d matrix; Eigen::Vector3d old_bias;
  if (!parseCompensation(device_values_.value(id), &matrix, &old_bias)) { setStatus(QString("Read register %1 before staging calibration").arg(id), true); emit requestRead(id); return; }
  Eigen::Matrix3d output_rotation; Eigen::Vector3d unused;
  if (!parseCompensation(device_values_.value(26) + ";0;0;0", &output_rotation, &unused)) {
    setStatus("Read reference-frame rotation register 26 before staging calibration", true);
    emit requestRead(26); return;
  }
  if (calibration_step_ == 0) {
    if (!device_values_.contains(43)) { setStatus("Read startup gyro bias register 43 before staging gyro compensation", true); emit requestRead(43); return; }
    Eigen::Vector3d startup; const auto fields=device_values_[43].split(';'); bool ok=fields.size()==3;
    for(int i=0;i<3 && ok;++i){bool part=false; startup(i)=fields[i].toDouble(&part);ok&=part;}
    if (!ok || startup.norm() > 1e-7) { setStatus("Gyro staging blocked: nonzero or unreadable startup filter bias could double-correct angular rate", true); return; }
    staged_values_[84] = formatCompensation(matrix, composeCompensationBias(matrix, output_rotation, old_bias, gyro_result_.residual_bias));
  } else if (calibration_step_ == 1) {
    staged_values_[25] = formatCompensation(matrix, composeCompensationBias(matrix, output_rotation, old_bias, accel_result_.bias));
  } else {
    staged_values_[23] = formatCompensation(matrix, composeCompensationBias(matrix, output_rotation, old_bias, mag_result_.in_plane_bias));
  }
  refreshRegisterTable(); setStatus("Calibration result staged; inspect it before applying");
}

void MainWindow::exportCalibrationReport() {
  const QString path = QFileDialog::getSaveFileName(this, "Export calibration report", {}, "JSON (*.json)"); if (path.isEmpty()) return;
  QJsonObject report{{"schema_version", 1}, {"warning", "Gravity-derived tilt is not sensor-to-robot mounting rotation; yaw is unobservable."},
                     {"gravity_sign", "gravity_vector_sensor = -mean_stationary_specific_force"}, {"frame", "VectorNav configured sensor output frame"}};
  auto vec=[](const Eigen::Vector3d& v){return QJsonArray{v.x(),v.y(),v.z()};};
  report["gyro"] = QJsonObject{{"evaluated",!gyro_result_.quality.summary.empty()},{"sample_count",static_cast<int>(gyro_samples_.size())},{"passed",gyro_result_.quality.passed},{"residual_bias_rad_s",vec(gyro_result_.residual_bias)},{"quality",QString::fromStdString(gyro_result_.quality.summary)}};
  report["accelerometer"] = QJsonObject{{"evaluated",!accel_result_.quality.summary.empty()},{"sample_count",static_cast<int>(accel_samples_.size())},{"passed",accel_result_.quality.passed},{"bias_m_s2",vec(accel_result_.bias)},{"measured_magnitude_m_s2",accel_result_.measured_gravity},{"quality",QString::fromStdString(accel_result_.quality.summary)}};
  report["magnetometer"] = QJsonObject{{"evaluated",!mag_result_.quality.summary.empty()},{"sample_count",static_cast<int>(mag_samples_.size())},{"paired_gravity_sample_count",static_cast<int>(mag_accel_samples_.size())},{"passed",mag_result_.quality.passed},{"in_plane_bias_gauss",vec(mag_result_.in_plane_bias)},{"coverage_deg",mag_result_.coverage_deg},{"quality",QString::fromStdString(mag_result_.quality.summary)}};
  report["gravity_report"] = QJsonObject{{"evaluated",!gravity_report_.quality.summary.empty()},{"sample_count",static_cast<int>(gravity_samples_.size())},{"passed",gravity_report_.quality.passed},{"specific_force_mean_m_s2",vec(gravity_report_.specific_force_mean)},{"gravity_vector_m_s2",vec(gravity_report_.gravity_vector)},{"magnitude_m_s2",gravity_report_.magnitude},{"tilt_from_sensor_z_deg",gravity_report_.tilt_from_sensor_z_deg},{"quality",QString::fromStdString(gravity_report_.quality.summary)}};
  report["policy"] = QJsonObject{{"minimum_samples",static_cast<int>(policy_.minimum_samples)},
      {"gyro_max_rms_rad_s",policy_.gyro_max_rms_rad_s},{"gyro_max_temperature_span_c",policy_.gyro_max_temperature_span_c},
      {"gyro_max_pose_delta_rad_s",policy_.gyro_max_pose_delta_rad_s},{"accelerometer_max_rms_m_s2",policy_.accel_max_rms_m_s2},
      {"accelerometer_max_condition",policy_.accel_max_condition},{"accelerometer_min_axis_span_m_s2",policy_.accel_min_axis_span_m_s2},
      {"magnetometer_max_rms_fraction",policy_.mag_max_rms_fraction},{"magnetometer_min_coverage_deg",policy_.mag_min_coverage_deg},
      {"magnetometer_max_tilt_stddev_deg",policy_.mag_max_tilt_stddev_deg}};
  QFile file(path); if (!file.open(QIODevice::WriteOnly|QIODevice::Truncate)) { setStatus(file.errorString(),true);return;} file.write(QJsonDocument(report).toJson(QJsonDocument::Indented)); setStatus("Calibration report exported");
}

void MainWindow::exportTelemetry() {
  const QString path = QFileDialog::getSaveFileName(this, "Export telemetry", {}, "CSV (*.csv)");
  if (path.isEmpty()) return;
  QFile file(path);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) { setStatus(file.errorString(), true); return; }
  file.write("time_s,qx,qy,qz,qw,mx,my,mz,ax,ay,az,gx,gy,gz,temperature_c,pressure_kpa\n");
  for (const auto& s : recorded_telemetry_) {
    const QString line = QString("%1,%2,%3,%4,%5,%6,%7,%8,%9,%10,%11,%12,%13,%14,%15,%16\n")
        .arg(s.time_s,0,'g',16).arg(s.qx,0,'g',12).arg(s.qy,0,'g',12).arg(s.qz,0,'g',12).arg(s.qw,0,'g',12)
        .arg(s.mx,0,'g',12).arg(s.my,0,'g',12).arg(s.mz,0,'g',12).arg(s.ax,0,'g',12).arg(s.ay,0,'g',12)
        .arg(s.az,0,'g',12).arg(s.gx,0,'g',12).arg(s.gy,0,'g',12).arg(s.gz,0,'g',12)
        .arg(s.temperature_c,0,'g',12).arg(s.pressure_kpa,0,'g',12);
    file.write(line.toUtf8());
  }
  setStatus(QString("Exported %1 telemetry samples").arg(recorded_telemetry_.size()));
}

void MainWindow::loadReplay() {
  const QString path = QFileDialog::getOpenFileName(this, "Replay telemetry", {}, "CSV (*.csv)");
  if (path.isEmpty()) return;
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) { setStatus(file.errorString(), true); return; }
  replay_telemetry_.clear();
  file.readLine();
  while (!file.atEnd()) {
    const auto fields = QString::fromUtf8(file.readLine()).trimmed().split(',');
    if (fields.size() != 16) continue;
    Telemetry s; bool ok = true; double* values[] = {&s.time_s,&s.qx,&s.qy,&s.qz,&s.qw,&s.mx,&s.my,&s.mz,&s.ax,&s.ay,&s.az,&s.gx,&s.gy,&s.gz,&s.temperature_c,&s.pressure_kpa};
    for (int i=0;i<16;++i) { bool field_ok=false; *values[i]=fields[i].toDouble(&field_ok); ok &= field_ok; }
    if (ok) replay_telemetry_.push_back(s);
  }
  if (replay_telemetry_.empty()) { setStatus("Replay file has no valid telemetry rows", true); return; }
  if (!replay_timer_) {
    replay_timer_ = new QTimer(this); replay_timer_->setInterval(50);
    connect(replay_timer_, &QTimer::timeout, this, [this] {
      if (replay_telemetry_.empty()) { replay_timer_->stop(); setStatus("Replay complete"); return; }
      const bool was_recording = recording_; recording_ = false; onTelemetry(replay_telemetry_.front()); recording_ = was_recording; replay_telemetry_.pop_front();
    });
  }
  replay_timer_->start(); setStatus("Replaying telemetry through monitor pipeline");
}

}  // namespace riptide_imu
