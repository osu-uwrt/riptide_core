#include "riptide_imu/config_tool/main_window.hpp"
#include "riptide_imu/config_tool/calibration_wizard.hpp"
#include "riptide_imu/config_tool/ros_sensor_transport.hpp"
#include "riptide_imu/config_tool/theme.hpp"

#include <QAction>
#include <QActionGroup>
#include <QApplication>
#include <QComboBox>
#include <QCompleter>
#include <QDateTime>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSettings>
#include <QSignalBlocker>
#include <QSplitter>
#include <QStackedWidget>
#include <QStatusBar>
#include <QStyle>
#include <QTabWidget>
#include <QTableWidget>
#include <QTimer>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <cmath>
#include <functional>

namespace riptide_imu {
namespace {

const QStringList kModes{"ros", "serial", "demo"};
const int kCalibrationRegisters[] = {23, 25, 26, 43, 84};
const int kOutputRegisters[] = {5, 6, 7, 75, 76, 77};
// Reading this one resets the sensor's integration interval, so it is never read just because it was selected.
const int kNoAutoRead[] = {80};

QTabWidget* panel(QWidget* page, const QString& title, const QString& object_name) {
  auto* tabs = new QTabWidget;
  tabs->setObjectName(object_name);
  tabs->setDocumentMode(true);
  tabs->addTab(page, title);
  return tabs;
}

QWidget* sheet() {
  auto* widget = new QWidget;
  widget->setObjectName("sheet");
  widget->setAttribute(Qt::WA_StyledBackground);
  return widget;
}

void repolish(QWidget* widget) {
  widget->style()->unpolish(widget);
  widget->style()->polish(widget);
}

QPushButton* commandButton(const QString& text, const QString& tooltip, QAction* action = nullptr) {
  auto* button = new QPushButton(text);
  button->setToolTip(tooltip);
  if (action) QObject::connect(button, &QPushButton::clicked, action, &QAction::trigger);
  return button;
}

// A click anywhere in an editable combo box refreshes its entries (when `refresh` is given) and opens the whole
// list; typing then narrows it to matching entries.
class ListOnClick : public QObject {
 public:
  ListOnClick(QComboBox* combo, std::function<void()> refresh)
      : QObject(combo), combo_(combo), refresh_(std::move(refresh)) {}

 protected:
  bool eventFilter(QObject*, QEvent* event) override {
    if (event->type() != QEvent::MouseButtonRelease || !combo_->isEnabled() ||
        combo_->completer()->popup()->isVisible())
      return false;
    if (refresh_) refresh_();
    if (combo_->count() > 0) {
      combo_->completer()->setCompletionPrefix(QString());
      combo_->completer()->complete();
    }
    return false;
  }

 private:
  QComboBox* combo_;
  std::function<void()> refresh_;
};

void openListOnClick(QComboBox* combo, std::function<void()> refresh = {}) {
  auto* completer = new QCompleter(combo->model(), combo);
  completer->setFilterMode(Qt::MatchContains);
  completer->setCaseSensitivity(Qt::CaseInsensitive);
  completer->setCompletionMode(QCompleter::PopupCompletion);
  completer->setMaxVisibleItems(12);
  combo->setCompleter(completer);
  combo->lineEdit()->installEventFilter(new ListOnClick(combo, std::move(refresh)));
}

}  // namespace

MainWindow::MainWindow(QString imu_port, int baud, QString config_file, bool demo,
                       QString share_directory, QWidget* parent,
                       SensorInterface* injected_sensor, QString connection_mode)
    : QMainWindow(parent), share_directory_(std::move(share_directory)),
      connection_mode_(demo ? "demo" : connection_mode.toLower()) {
  qRegisterMetaType<Telemetry>();
  setWindowTitle("VectorNav Workshop");
  setObjectName("vectorNavWorkbench");
  resize(1440, 900);

  auto* central = new QWidget;
  auto* layout = new QVBoxLayout(central);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(0);
  layout->addWidget(buildBoard(imu_port, baud));
  layout->addWidget(buildCommandBar());

  main_splitter_ = new QSplitter(Qt::Horizontal);
  main_splitter_->setObjectName("mainSplitter");
  main_splitter_->setChildrenCollapsible(false);
  main_splitter_->addWidget(panel(buildRegisterListPage(), "Registers", "registerPanel"));
  center_splitter_ = new QSplitter(Qt::Vertical);
  center_splitter_->setObjectName("centerSplitter");
  center_splitter_->setChildrenCollapsible(false);
  center_splitter_->addWidget(panel(buildInspectorPage(), "Register", "inspectorPanel"));
  lower_tabs_ = new QTabWidget;
  lower_tabs_->setObjectName("lowerPanel");
  lower_tabs_->setDocumentMode(true);
  lower_tabs_->addTab(buildSignalsPage(), "Signals");
  lower_tabs_->addTab(buildCalibrationPage(), "Calibration");
  lower_tabs_->addTab(buildProtocolPage(), "Protocol log");
  center_splitter_->addWidget(lower_tabs_);
  main_splitter_->addWidget(center_splitter_);
  right_splitter_ = new QSplitter(Qt::Vertical);
  right_splitter_->setObjectName("rightSplitter");
  right_splitter_->setChildrenCollapsible(false);
  right_splitter_->addWidget(panel(buildAttitudePage(), "Attitude", "attitudePanel"));
  telemetry_summary_ = new TelemetrySummaryWidget;
  telemetry_summary_->setWorldFrame(connection_mode_ == "ros" ? AttitudeWidget::WorldFrame::Enu
                                                              : AttitudeWidget::WorldFrame::Ned);
  auto* live_page = sheet();
  auto* live_layout = new QVBoxLayout(live_page);
  live_layout->setContentsMargins(0, 0, 0, 8);
  auto* live_scroll = new QScrollArea;
  live_scroll->setWidget(telemetry_summary_);
  live_scroll->setWidgetResizable(true);
  live_scroll->setFrameShape(QFrame::NoFrame);
  live_scroll->viewport()->setAutoFillBackground(false);
  live_layout->addWidget(live_scroll, 1);
  auto* live_controls = new QHBoxLayout;
  live_controls->setContentsMargins(12, 0, 12, 0);
  live_controls->addStretch();
  auto* restart_drift = new QPushButton("Restart drift measurement");
  restart_drift->setObjectName("restartDriftButton");
  restart_drift->setToolTip("Start the drift total and rate from now");
  connect(restart_drift, &QPushButton::clicked, telemetry_summary_, &TelemetrySummaryWidget::restartDrift);
  live_controls->addWidget(restart_drift);
  live_layout->addLayout(live_controls);
  right_splitter_->addWidget(panel(live_page, "Live data", "liveDataPanel"));
  main_splitter_->addWidget(right_splitter_);
  main_splitter_->setStretchFactor(1, 1);
  layout->addWidget(main_splitter_, 1);
  setCentralWidget(central);
  buildMenus();

  status_ = new QLabel("Disconnected");
  status_->setObjectName("statusMessage");
  statusBar()->addWidget(status_, 1);
  statusBar()->setSizeGripEnabled(false);
  loadDefinitions(share_directory_ + "/config/registers.json");
  // before any connection, which can come up during construction
  auto_read_timer_ = new QTimer(this);
  auto_read_timer_->setSingleShot(true);
  auto_read_timer_->setInterval(150);
  connect(auto_read_timer_, &QTimer::timeout, this, [this] {
    if (connected_ && connection_mode_ != "demo" && current_register_id_ >= 0) emit requestRead(current_register_id_);
  });

  try {
    policy_ = loadCalibrationPolicy((share_directory_ + "/config/calibration_policy.json").toStdString());
  } catch (const std::exception& exception) {
    setStatus("Calibration policy: " + QString::fromStdString(exception.what()), true);
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
  connect(transport_, &SensorInterface::calibrationSampleReceived, this, &MainWindow::onCalibrationSample);
  connect(transport_, &SensorInterface::registerRead, this, &MainWindow::onRegisterRead);
  connect(transport_, &SensorInterface::writeVerified, this, &MainWindow::onRegisterRead);
  connect(transport_, &SensorInterface::connectionChanged, this, [this](bool ok, const QString& detail) {
    connected_ = ok;
    connection_button_->setEnabled(true);
    connection_button_->setText(ok ? "Disconnect" : "Connect");
    connection_button_->setProperty("role", ok ? "" : "primary");
    repolish(connection_button_);
    port_selector_->setEnabled(!ok);
    baud_selector_->setEnabled(!ok);
    connection_mode_selector_->setEnabled(!ok);
    connected_target_ = ok ? (connection_mode_ == "demo" ? QString("demo") : port_selector_->currentText())
                           : QString();
    // long by-id paths cut short, full one in the tooltip
    const QString context = connected_target_.isEmpty() ? QString("/  VN-100")
                                                        : QString("/  VN-100 · %1").arg(connected_target_);
    context_label_->setText(context_label_->fontMetrics().elidedText(context, Qt::ElideMiddle, 220));
    context_label_->setToolTip(context);
    if (!ok) {
      attitude_->clearAttitude();
      telemetry_summary_->markStale(true);
    }
    bandwidth_meter_->setVisible(ok && connection_mode_ != "demo");
    if (ok) {
      readOutputRegisters();
      // register on show gets read like any selection
      scheduleAutoRead();
    }
    if (wizard_) wizard_->setConnected(ok);
    refreshTelemetryChip();
    refreshBandwidth();
    setStatus(detail, !ok && detail != "Disconnected");
  });
  connect(transport_, &SensorInterface::operationFailed, this, [this](const QString& op, const QString& detail) {
    setStatus(op + ": " + detail, true);
    if (op == "Enter maintenance") {
      after_maintenance_ = {};
      QMessageBox::warning(this, "Could not enter maintenance", "The driver refused maintenance mode: " + detail);
    } else if (detail.contains("requires maintenance mode")) {
      // the driver's own check, e.g. after the lease lapsed
      askForMaintenance(op, {});
    }
  });
  connect(transport_, &SensorInterface::operationSucceeded, this,
          [this](const QString& op, const QString& detail) { setStatus(op + ": " + detail); });
  connect(transport_, &SensorInterface::maintenanceChanged, this, [this](bool active, const QString& detail) {
    maintenance_active_ = active;
    maintenance_button_->setText(active ? "Leave maintenance" : "Enter maintenance");
    maintenance_chip_->setVisible(active);
    setStatus(detail);
    auto next = std::move(after_maintenance_);
    after_maintenance_ = {};
    if (active && next) next();
  });
  connect(transport_, &SensorInterface::endpointsDiscovered, this, [this, baud](const QStringList& endpoints) {
    if (connection_mode_ != "ros") return;
    const QString selected = port_selector_->currentText();
    port_selector_->clear();
    port_selector_->addItems(endpoints);
    if (!selected.isEmpty()) port_selector_->setCurrentText(selected);
    if (port_selector_->completer()->popup()->isVisible()) port_selector_->completer()->complete();
    // refresh only updates the list, never connects
    if (endpoints.size() == 1 && !connected_ && discovery_auto_connect_) {
      port_selector_->setCurrentIndex(0);
      connection_button_->setEnabled(false);
      emit requestConnect(endpoints.front(), baud);
    } else {
      connection_button_->setEnabled(true);
      setStatus(endpoints.isEmpty() ? "No VectorNav drivers discovered" : "Select a robot endpoint");
    }
  });
  connect(transport_, &SensorInterface::traffic, this, [this](const QString& direction, const QString& packet) {
    traffic_log_->appendPlainText(QTime::currentTime().toString("HH:mm:ss.zzz") + "  " + direction + "  " + packet);
  });
  connect(telemetry_summary_, &TelemetrySummaryWidget::freshnessChanged, this, [this] { refreshTelemetryChip(); });

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
    QString error;
    configuration_metadata_ = loadControlCenterXml(config_file, &error);
    for (const auto& reg : configuration_metadata_.registers) staged_values_[reg.id] = reg.values;
    refreshRegisterTable();
    if (!error.isEmpty()) setStatus("Configuration import: " + error, true);
  }
  last_sample_clock_.start();
  stale_timer_ = new QTimer(this);
  stale_timer_->setInterval(500);
  connect(stale_timer_, &QTimer::timeout, this,
          [this] { telemetry_summary_->markStale(last_sample_clock_.elapsed() > 1200); });
  stale_timer_->start();
  refreshCalibrationSummary();
  refreshTelemetryChip();
  restoreLayout();
}

MainWindow::~MainWindow() {
  QSettings settings;
  settings.setValue("workbench/geometry", saveGeometry());
  settings.setValue("workbench/main", main_splitter_->saveState());
  settings.setValue("workbench/center", center_splitter_->saveState());
  settings.setValue("workbench/right", right_splitter_->saveState());
  settings.setValue("workbench/lowerTab", lower_tabs_->currentIndex());
  settings.sync();
  delete wizard_;
  if (owns_transport_) {
    QMetaObject::invokeMethod(transport_, "disconnectSensor", Qt::BlockingQueuedConnection);
    sensor_thread_.quit();
    sensor_thread_.wait(3000);
  }
}

QWidget* MainWindow::buildBoard(const QString& initial_port, int initial_baud) {
  auto* board = new QWidget;
  board->setObjectName("board");
  board->setAttribute(Qt::WA_StyledBackground);
  auto* layout = new QHBoxLayout(board);
  layout->setContentsMargins(16, 8, 14, 8);
  layout->setSpacing(10);

  auto* wordmark = new QLabel("VECTORNAV");
  wordmark->setObjectName("wordmark");
  layout->addWidget(wordmark);
  context_label_ = new QLabel("/  VN-100");
  context_label_->setProperty("tone", "muted");
  context_label_->setMinimumWidth(80);
  layout->addWidget(context_label_);
  layout->addSpacing(14);

  connection_mode_selector_ = new SegmentedSwitch({"Robot (ROS)", "Local serial", "Demo"}, true);
  connection_mode_selector_->setObjectName("connectionModeSelector");
  connection_mode_selector_->setToolTip("Where the sensor is: behind the robot's ROS driver, on a local "
                                        "serial port, or generated demo data");
  connection_mode_selector_->setCurrentIndex(std::max(0, static_cast<int>(kModes.indexOf(connection_mode_))));
  layout->addWidget(connection_mode_selector_);

  connection_target_label_ = new QLabel(connection_mode_ == "ros" ? "ROBOT" : "PORT");
  connection_target_label_->setProperty("tone", "muted");
  layout->addWidget(connection_target_label_);
  port_selector_ = new QComboBox;
  port_selector_->setObjectName("sensorPortSelector");
  port_selector_->setAccessibleName("Sensor serial port");
  port_selector_->setEditable(true);
  port_selector_->setInsertPolicy(QComboBox::NoInsert);
  port_selector_->setMinimumContentsLength(12);
  port_selector_->setToolTip("A discovered robot namespace, or a local serial device path");
  if (!initial_port.trimmed().isEmpty()) port_selector_->addItem(initial_port.trimmed());
  if (connection_mode_ != "ros") refreshSerialPorts();
  port_selector_->setCurrentText(initial_port);
  // rescan on every click
  openListOnClick(port_selector_, [this] { refreshConnectionTargets(false); });
  layout->addWidget(port_selector_, 1);

  auto* refresh = new QPushButton("Rescan");
  refresh->setObjectName("refreshSerialPortsButton");
  refresh->setToolTip("Rediscover ROS drivers or rescan Linux serial devices");
  connect(refresh, &QPushButton::clicked, this, [this] { refreshConnectionTargets(); });
  layout->addWidget(refresh);

  baud_label_ = new QLabel("BAUD");
  baud_label_->setProperty("tone", "muted");
  layout->addWidget(baud_label_);
  baud_selector_ = new QComboBox;
  baud_selector_->setObjectName("sensorBaudSelector");
  baud_selector_->setAccessibleName("Sensor baud rate");
  baud_selector_->setEditable(true);
  baud_selector_->addItems({"9600", "19200", "38400", "57600", "115200", "128000", "230400", "460800", "921600"});
  baud_selector_->setCurrentText(QString::number(initial_baud));
  baud_selector_->setMinimumContentsLength(7);
  openListOnClick(baud_selector_);
  layout->addWidget(baud_selector_);
  baud_selector_->setVisible(connection_mode_ != "ros");
  baud_label_->setVisible(connection_mode_ != "ros");

  connection_button_ = new QPushButton("Connect");
  connection_button_->setObjectName("connectButton");
  connection_button_->setAccessibleName("Connect or disconnect sensor");
  connection_button_->setProperty("role", "primary");
  connection_button_->setMinimumWidth(96);
  connect(connection_button_, &QPushButton::clicked, this, &MainWindow::toggleConnection);
  layout->addWidget(connection_button_);
  layout->addStretch();

  staged_chip_ = new StatusChip("0 staged", StatusChip::Level::Change, true);
  staged_chip_->setObjectName("stagedChip");
  staged_chip_->setToolTip("Values staged in the workspace that the sensor does not have yet");
  staged_chip_->hide();
  layout->addWidget(staged_chip_);
  maintenance_chip_ = new StatusChip("Maintenance", StatusChip::Level::Warn, true);
  maintenance_chip_->setToolTip("Driver-critical writes, reset and flash save are allowed");
  maintenance_chip_->hide();
  layout->addWidget(maintenance_chip_);
  telemetry_chip_ = new StatusChip("Offline", StatusChip::Level::Neutral, true);
  telemetry_chip_->setObjectName("telemetryChip");
  layout->addWidget(telemetry_chip_);

  connect(connection_mode_selector_, &SegmentedSwitch::currentIndexChanged, this,
          [this](int index) { setConnectionMode(kModes.value(index, "ros")); });
  return board;
}

QWidget* MainWindow::buildCommandBar() {
  auto* bar = new QWidget;
  bar->setObjectName("commandBar");
  bar->setAttribute(Qt::WA_StyledBackground);
  auto* layout = new QHBoxLayout(bar);
  layout->setContentsMargins(12, 7, 12, 7);
  layout->setSpacing(6);

  auto* read = commandButton("Read", "Read the selected register from the sensor (F5)");
  connect(read, &QPushButton::clicked, this, &MainWindow::readSelectedRegister);
  layout->addWidget(read);
  apply_button_ = commandButton("Apply", "Write every staged value and read it back (Ctrl+Return)");
  apply_button_->setObjectName("applyButton");
  connect(apply_button_, &QPushButton::clicked, this, &MainWindow::applyStagedRegisters);
  layout->addWidget(apply_button_);
  auto* discard = commandButton("Discard", "Drop all staged values; the sensor is unchanged");
  discard->setObjectName("discardButton");
  connect(discard, &QPushButton::clicked, this, &MainWindow::discardStagedRegisters);
  layout->addWidget(discard);
  layout->addSpacing(14);
  auto* import_button = commandButton("Import XML…", "Stage a VectorNav Control Center configuration");
  connect(import_button, &QPushButton::clicked, this, &MainWindow::importConfiguration);
  layout->addWidget(import_button);
  auto* export_button = commandButton("Export XML…", "Write device and staged values as Control Center XML");
  connect(export_button, &QPushButton::clicked, this, &MainWindow::exportConfiguration);
  layout->addWidget(export_button);
  layout->addSpacing(14);
  auto* save = commandButton("Save to flash…", "Persist the active configuration, reset and verify");
  connect(save, &QPushButton::clicked, this, &MainWindow::savePersistently);
  layout->addWidget(save);
  // next to the actions it can block
  layout->addSpacing(18);
  bandwidth_meter_ = new BandwidthMeter;
  bandwidth_meter_->hide();
  layout->addWidget(bandwidth_meter_);
  layout->addStretch();

  maintenance_button_ = commandButton("Enter maintenance", "Required on the robot for settings that can "
                                                           "interrupt the live driver");
  maintenance_button_->setObjectName("maintenanceButton");
  connect(maintenance_button_, &QPushButton::clicked, this, [this] {
    if (maintenance_active_) {
      emit requestCloseMaintenance();
      return;
    }
    if (confirm("Enter sensor maintenance",
                "Driver-critical writes, reset, and persistent save can briefly interrupt IMU data. "
                "The driver process will remain running. Enter maintenance mode?", true))
      emit requestOpenMaintenance();
  });
  layout->addWidget(maintenance_button_);
  auto* calibrate = commandButton("Calibrate…", "Open the guided calibration");
  calibrate->setObjectName("calibrateButton");
  calibrate->setProperty("role", "primary");
  connect(calibrate, &QPushButton::clicked, this, &MainWindow::openCalibrationWizard);
  layout->addWidget(calibrate);
  return bar;
}

void MainWindow::buildMenus() {
  auto* file_menu = menuBar()->addMenu("&File");
  file_menu->addAction("Import Control Center XML…", this, &MainWindow::importConfiguration);
  file_menu->addAction("Export Control Center XML…", this, &MainWindow::exportConfiguration);
  file_menu->addSeparator();
  file_menu->addAction("Export telemetry CSV…", this, &MainWindow::exportTelemetry);
  file_menu->addAction("Replay telemetry CSV…", this, &MainWindow::loadReplay);
  file_menu->addAction("Export calibration report…", this, &MainWindow::exportCalibrationReport);
  file_menu->addAction("Export protocol log…", this, &MainWindow::exportProtocolLog);
  file_menu->addSeparator();
  file_menu->addAction("Quit", this, &QWidget::close, QKeySequence::Quit);

  auto* sensor_menu = menuBar()->addMenu("&Sensor");
  sensor_menu->addAction("Read selected register", this, &MainWindow::readSelectedRegister, QKeySequence::Refresh);
  sensor_menu->addAction("Apply staged values", this, &MainWindow::applyStagedRegisters,
                         QKeySequence("Ctrl+Return"));
  sensor_menu->addAction("Discard staged values", this, &MainWindow::discardStagedRegisters);
  sensor_menu->addAction("Save active settings to flash…", this, &MainWindow::savePersistently);
  sensor_menu->addSeparator();
  sensor_menu->addAction("Calibrate…", this, &MainWindow::openCalibrationWizard, QKeySequence("Ctrl+K"));
  sensor_menu->addSeparator();
  sensor_menu->addAction("Tare orientation…", this, [this] {
    if (confirm("Tare orientation", "Tare changes the sensor's active attitude reference. Continue?"))
      emit requestTare();
  });
  sensor_menu->addAction("Reset sensor…", this, &MainWindow::resetSensor);

  auto* view_menu = menuBar()->addMenu("&View");
  auto* theme_menu = view_menu->addMenu("Theme");
  auto* group = new QActionGroup(this);
  for (const QString& id : themeIds()) {
    auto* action = theme_menu->addAction(themeLabel(id));
    action->setCheckable(true);
    action->setChecked(theme().id == id);
    action->setData(id);
    group->addAction(action);
    theme_actions_ << action;
    connect(action, &QAction::triggered, this, [this, id] { setTheme(id); });
  }
  view_menu->addSeparator();
  auto* record = view_menu->addAction("Record telemetry in memory");
  record->setCheckable(true);
  connect(record, &QAction::toggled, this, [this](bool enabled) {
    recording_ = enabled;
    setStatus(enabled ? "Recording telemetry in memory" : "Recording stopped");
  });
  view_menu->addSeparator();
  view_menu->addAction("Reset layout", this, &MainWindow::resetDashboardLayout);

  auto* search = new QAction(this);
  search->setShortcut(QKeySequence::Find);
  addAction(search);
  connect(search, &QAction::triggered, register_filter_, qOverload<>(&QWidget::setFocus));
}

QWidget* MainWindow::buildRegisterListPage() {
  auto* page = sheet();
  auto* layout = new QVBoxLayout(page);
  layout->setContentsMargins(10, 10, 10, 6);
  layout->setSpacing(8);
  register_filter_ = new QLineEdit;
  register_filter_->setObjectName("registerSearch");
  register_filter_->setClearButtonEnabled(true);
  register_filter_->setPlaceholderText("Search ID, name or description  (Ctrl+F)");
  layout->addWidget(register_filter_);
  register_tree_ = new QTreeWidget;
  register_tree_->setObjectName("registerTree");
  register_tree_->setColumnCount(2);
  register_tree_->setHeaderLabels({"Register", "State"});
  register_tree_->setRootIsDecorated(true);
  register_tree_->setIndentation(14);
  register_tree_->setUniformRowHeights(true);
  register_tree_->header()->setStretchLastSection(false);
  register_tree_->header()->setSectionResizeMode(0, QHeaderView::Stretch);
  register_tree_->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  layout->addWidget(register_tree_, 1);
  connect(register_filter_, &QLineEdit::textChanged, this, [this] { refreshRegisterTable(); });
  connect(register_tree_, &QTreeWidget::currentItemChanged, this, &MainWindow::selectRegister);
  connect(register_tree_, &QTreeWidget::itemClicked, this, [this](QTreeWidgetItem* item) {
    if (item && item->parent() && item->data(0, Qt::UserRole).toInt() == current_register_id_) scheduleAutoRead();
  });
  return page;
}

QWidget* MainWindow::buildInspectorPage() {
  auto* page = sheet();
  auto* layout = new QVBoxLayout(page);
  layout->setContentsMargins(16, 12, 16, 12);
  layout->setSpacing(8);
  register_title_ = sectionTitle("No register selected");
  layout->addWidget(register_title_);
  auto* chips = new QHBoxLayout;
  chips->setSpacing(6);
  register_group_chip_ = new StatusChip("", StatusChip::Level::Neutral);
  register_access_chip_ = new StatusChip("", StatusChip::Level::Neutral);
  register_state_chip_ = new StatusChip("", StatusChip::Level::Neutral);
  register_state_chip_->setObjectName("registerStateChip");
  chips->addWidget(register_group_chip_);
  chips->addWidget(register_access_chip_);
  chips->addWidget(register_state_chip_);
  chips->addStretch();
  layout->addLayout(chips);
  register_description_ = toneLabel("Select a register to compare its device and staged values.", "muted");
  register_description_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  layout->addWidget(register_description_);

  register_inspector_ = new QTableWidget(0, 4);
  register_inspector_->setObjectName("registerInspector");
  register_inspector_->setHorizontalHeaderLabels({"Field", "Device", "Staged", "Units"});
  register_inspector_->setAlternatingRowColors(true);
  register_inspector_->setShowGrid(false);
  register_inspector_->verticalHeader()->setVisible(false);
  register_inspector_->verticalHeader()->setDefaultSectionSize(32);
  auto* header = register_inspector_->horizontalHeader();
  header->setDefaultAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  header->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  header->setSectionResizeMode(1, QHeaderView::Stretch);
  header->setSectionResizeMode(2, QHeaderView::Stretch);
  header->setSectionResizeMode(3, QHeaderView::ResizeToContents);
  binary_editor_ = new BinaryOutputEditor;
  auto* binary_scroll = new QScrollArea;
  binary_scroll->setWidget(binary_editor_);
  binary_scroll->setWidgetResizable(true);
  binary_scroll->setFrameShape(QFrame::NoFrame);
  binary_scroll->viewport()->setAutoFillBackground(false);
  inspector_stack_ = new QStackedWidget;
  inspector_stack_->addWidget(register_inspector_);
  inspector_stack_->addWidget(binary_scroll);
  layout->addWidget(inspector_stack_, 1);

  auto* raw_row = new QHBoxLayout;
  auto* raw_label = new QLabel("RAW");
  raw_label->setProperty("tone", "muted");
  raw_label->setToolTip("The register's protocol fields, separated by semicolons");
  raw_row->addWidget(raw_label);
  raw_register_editor_ = new QLineEdit;
  raw_register_editor_->setObjectName("rawRegisterEditor");
  raw_register_editor_->setPlaceholderText("Semicolon-separated register fields");
  raw_row->addWidget(raw_register_editor_, 1);
  stage_raw_button_ = new QPushButton("Stage raw");
  raw_row->addWidget(stage_raw_button_);
  layout->addLayout(raw_row);

  connect(register_inspector_, &QTableWidget::cellChanged, this, [this](int, int column) {
    if (column == 2) stageInspectorValues();
  });
  connect(binary_editor_, &BinaryOutputEditor::edited, this, [this](const QString& values) {
    if (current_register_id_ < 0) return;
    const auto definition = definitionForId(current_register_id_);
    QString error;
    if (!validateRegisterValue(definition, values, &error)) {
      setStatus(definition.name + ": " + error, true);
      return;
    }
    if (registerValuesMatch(values, device_values_.value(current_register_id_)))
      staged_values_.remove(current_register_id_);
    else
      staged_values_[current_register_id_] = values;
    refreshRegisterTable();
    setStatus(definition.name + " staged; sensor unchanged");
  });
  connect(stage_raw_button_, &QPushButton::clicked, this, [this] {
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
  return page;
}

QWidget* MainWindow::buildAttitudePage() {
  auto* page = sheet();
  auto* layout = new QVBoxLayout(page);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(0);
  auto* controls = new QHBoxLayout;
  controls->setContentsMargins(10, 8, 10, 6);
  auto* preset = new SegmentedSwitch({"3D", "Level", "Top"});
  preset->setObjectName("attitudeViewPreset");
  preset->setToolTip("3D: from above and behind. Level: side-on, to read tilt against the horizon ring. "
                     "Top: straight down, to read heading.");
  controls->addWidget(preset);
  controls->addStretch();
  auto* follow = new QPushButton("Follow heading");
  follow->setCheckable(true);
  follow->setToolTip("Keep the camera behind the sensor as it turns");
  controls->addWidget(follow);
  layout->addLayout(controls);
  attitude_ = new AttitudeWidget;
  attitude_->setWorldFrame(connection_mode_ == "ros" ? AttitudeWidget::WorldFrame::Enu
                                                     : AttitudeWidget::WorldFrame::Ned);
  layout->addWidget(attitude_, 1);
  connect(preset, &SegmentedSwitch::activated, attitude_, &AttitudeWidget::setViewPreset);
  // orbiting leaves the preset
  connect(attitude_, &AttitudeWidget::orbited, preset, [preset] {
    const QSignalBlocker blocker(preset);
    preset->setCurrentIndex(0);
  });
  connect(follow, &QPushButton::toggled, attitude_, &AttitudeWidget::setFollowHeading);
  return page;
}

QWidget* MainWindow::buildSignalsPage() {
  auto* page = sheet();
  auto* layout = new QVBoxLayout(page);
  layout->setContentsMargins(10, 8, 10, 6);
  layout->setSpacing(4);
  auto* controls = new QHBoxLayout;
  controls->setSpacing(6);
  auto* quantity = new SegmentedSwitch({"Attitude", "Angular rate", "Acceleration", "Magnetic"});
  quantity->setObjectName("signalQuantity");
  quantity->setToolTip("Attitude plots roll, pitch and yaw over time, so drift shows as a slope");
  controls->addWidget(quantity);
  controls->addStretch();
  auto* span = new SegmentedSwitch({"20 s", "2 min", "10 min", "30 min"});
  span->setObjectName("signalWindow");
  span->setToolTip("Time span shown. Long spans make slow drift visible");
  controls->addWidget(span);
  layout->addLayout(controls);
  auto* options = new QHBoxLayout;
  options->setSpacing(6);
  options->addStretch();
  auto* pause = new QPushButton("Pause");
  pause->setCheckable(true);
  auto* csv = new QPushButton("CSV");
  auto* csv_menu = new QMenu(csv);
  csv_menu->addAction("Export recorded telemetry…", this, &MainWindow::exportTelemetry);
  csv_menu->addAction("Replay a recording…", this, &MainWindow::loadReplay);
  csv->setMenu(csv_menu);
  csv->setToolTip("Export telemetry recorded in memory (View › Record telemetry), or replay a CSV");
  options->addWidget(pause);
  options->addWidget(csv);
  layout->addLayout(options);
  plot_ = new PlotWidget;
  plot_->setWorldFrame(connection_mode_ == "ros" ? AttitudeWidget::WorldFrame::Enu : AttitudeWidget::WorldFrame::Ned);
  layout->addWidget(plot_, 1);
  connect(quantity, &SegmentedSwitch::currentIndexChanged, plot_, &PlotWidget::setQuantity);
  connect(span, &SegmentedSwitch::currentIndexChanged, plot_, [this](int index) {
    const double spans[] = {20, 120, 600, 1800};
    plot_->setWindowSeconds(spans[index]);
  });
  connect(pause, &QPushButton::toggled, this, [this](bool paused) { monitor_paused_ = paused; });
  return page;
}

QWidget* MainWindow::buildCalibrationPage() {
  auto* page = sheet();
  auto* layout = new QHBoxLayout(page);
  layout->setContentsMargins(16, 12, 16, 12);
  layout->setSpacing(28);
  auto* left = new QVBoxLayout;
  left->setSpacing(10);
  left->addWidget(sectionTitle("Guided calibration"));
  left->addWidget(toneLabel(
      "Step-by-step capture of gyro bias, accelerometer bias, magnetometer hard iron and a gravity report, "
      "with drawings of each pose and live checks. Results are staged here for you to apply.", "muted"));
  auto* buttons = new QHBoxLayout;
  auto* open = new QPushButton("Open calibration…");
  open->setObjectName("openCalibrationButton");
  open->setProperty("role", "primary");
  connect(open, &QPushButton::clicked, this, &MainWindow::openCalibrationWizard);
  auto* report = new QPushButton("Export report…");
  connect(report, &QPushButton::clicked, this, &MainWindow::exportCalibrationReport);
  stage_latest_button_ = new QPushButton("Stage latest results");
  stage_latest_button_->setObjectName("stageLatestButton");
  stage_latest_button_->setToolTip("Stage every passed result that is not already staged or on the sensor");
  connect(stage_latest_button_, &QPushButton::clicked, this, &MainWindow::stageLatestCalibration);
  buttons->addWidget(open);
  buttons->addWidget(report);
  buttons->addStretch();
  left->addLayout(buttons);
  auto* stage_row = new QHBoxLayout;
  stage_row->addWidget(stage_latest_button_);
  stage_row->addStretch();
  left->addLayout(stage_row);
  calibration_note_ = toneLabel(QString(), "muted");
  calibration_note_->setObjectName("calibrationNote");
  left->addWidget(calibration_note_);
  left->addStretch();
  layout->addLayout(left, 2);
  auto* right = new QVBoxLayout;
  right->addWidget(sectionTitle("This session"));
  calibration_summary_ = new MetricTable;
  calibration_summary_->setObjectName("calibrationSummary");
  calibration_summary_->setColumns({"Step", "Result", "Register"}, {0.22, 0.6, 0.18});
  calibration_summary_->setFitContent(false);
  right->addWidget(calibration_summary_);
  right->addStretch();
  layout->addLayout(right, 3);
  return page;
}

QWidget* MainWindow::buildProtocolPage() {
  auto* page = sheet();
  auto* layout = new QVBoxLayout(page);
  layout->setContentsMargins(10, 6, 10, 8);
  traffic_log_ = new QPlainTextEdit;
  traffic_log_->setObjectName("protocolLog");
  traffic_log_->setReadOnly(true);
  traffic_log_->document()->setMaximumBlockCount(20000);
  layout->addWidget(traffic_log_, 1);
  auto* row = new QHBoxLayout;
  auto* tare = new QPushButton("Tare orientation…");
  auto* reset = new QPushButton("Reset sensor…");
  auto* export_log = new QPushButton("Export log…");
  export_log->setObjectName("exportLogButton");
  export_log->setToolTip("Save this log as a text file, with times");
  auto* clear = new QPushButton("Clear log");
  row->addWidget(tare);
  row->addWidget(reset);
  row->addStretch();
  row->addWidget(export_log);
  row->addWidget(clear);
  connect(export_log, &QPushButton::clicked, this, &MainWindow::exportProtocolLog);
  layout->addLayout(row);
  connect(clear, &QPushButton::clicked, traffic_log_, &QPlainTextEdit::clear);
  connect(tare, &QPushButton::clicked, this, [this] {
    if (confirm("Tare orientation", "Tare changes the sensor's active attitude reference. Continue?"))
      emit requestTare();
  });
  connect(reset, &QPushButton::clicked, this, &MainWindow::resetSensor);
  return page;
}

void MainWindow::applyDefaultLayout() {
  main_splitter_->setSizes({320, 650, 470});
  center_splitter_->setSizes({480, 300});
  right_splitter_->setSizes({400, 380});
  lower_tabs_->setCurrentIndex(0);
}

void MainWindow::restoreLayout() {
  QSettings settings;
  const QByteArray geometry = settings.value("workbench/geometry").toByteArray();
  if (!geometry.isEmpty()) restoreGeometry(geometry);
  applyDefaultLayout();
  main_splitter_->restoreState(settings.value("workbench/main").toByteArray());
  center_splitter_->restoreState(settings.value("workbench/center").toByteArray());
  right_splitter_->restoreState(settings.value("workbench/right").toByteArray());
  lower_tabs_->setCurrentIndex(settings.value("workbench/lowerTab", 0).toInt());
}

void MainWindow::resetDashboardLayout() {
  QSettings settings;
  settings.remove("workbench");
  resize(1440, 900);
  applyDefaultLayout();
  setStatus("Layout reset");
}

void MainWindow::setTheme(const QString& id) {
  applyTheme(qApp, id, share_directory_);
  QSettings().setValue("appearance/theme", id);
  for (auto* action : theme_actions_) action->setChecked(action->data().toString() == id);
  refreshRegisterTable();
}

void MainWindow::setConnectionMode(const QString& mode) {
  connection_mode_ = mode;
  maintenance_active_ = false;
  maintenance_button_->setText("Enter maintenance");
  maintenance_chip_->hide();
  connection_target_label_->setText(mode == "ros" ? "ROBOT" : "PORT");
  baud_selector_->setVisible(mode == "serial");
  baud_label_->setVisible(mode == "serial");
  port_selector_->setVisible(mode != "demo");
  connection_target_label_->setVisible(mode != "demo");
  const auto frame = mode == "ros" ? AttitudeWidget::WorldFrame::Enu : AttitudeWidget::WorldFrame::Ned;
  attitude_->setWorldFrame(frame);
  attitude_->clearAttitude();
  plot_->setWorldFrame(frame);
  telemetry_summary_->setWorldFrame(frame);
  emit requestConnectionMode(mode == "ros" ? "ros" : "serial");
  emit requestDemo(mode == "demo");
  port_selector_->clear();
  refreshConnectionTargets();
  connection_button_->setText("Connect");
  connection_button_->setEnabled(true);
}

void MainWindow::refreshConnectionTargets(bool auto_connect) {
  discovery_auto_connect_ = auto_connect;
  if (connection_mode_ == "ros")
    emit requestEndpointDiscovery();
  else
    refreshSerialPorts();
}

void MainWindow::refreshSerialPorts() {
  if (!port_selector_) return;
  const QString selected = port_selector_->currentText().trimmed();
  QStringList ports;
  if (!selected.isEmpty()) ports << selected;
  const QDir stable_directory("/dev/serial/by-id");
  for (const QString& entry :
       stable_directory.entryList(QDir::Files | QDir::System | QDir::NoDotAndDotDot, QDir::Name))
    ports << stable_directory.absoluteFilePath(entry);
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

void MainWindow::refreshTelemetryChip() {
  using Level = StatusChip::Level;
  const double output = bandwidth(false).output_rate_hz;
  const QString rate = output > 0 ? QString(" · %1 Hz").arg(output, 0, 'g', 4) : QString();
  if (!connected_) {
    telemetry_chip_->setState("Offline", Level::Neutral);
  } else if (telemetry_summary_->stale()) {
    telemetry_chip_->setState("No data" + rate, Level::Warn);
  } else {
    telemetry_chip_->setState("Live" + rate, Level::Ok);
  }
  telemetry_chip_->setToolTip(rateSource());
  telemetry_summary_->setSensorRate(connection_mode_ == "demo" ? 0.0 : output, rateSource());
}

QString MainWindow::rateSource() const {
  if (connection_mode_ == "demo") return "Demo data, generated at 20 Hz.";
  const double output = bandwidth(false).output_rate_hz;
  const QString sensor = output > 0
      ? QString("The sensor sends %1 Hz by itself (registers 6, 7 and 75-77).").arg(output, 0, 'g', 4)
      : QString("The sensor's own output rate is known once registers 6, 7 and 75-77 are read.");
  if (connection_mode_ == "ros")
    return sensor + " This view receives the driver's vectornav/imu topic at whatever rate it publishes.";
  return sensor + " Over local serial this view asks for attitude and IMU registers every 50 ms instead, so it "
                  "updates at about 20 Hz whatever the sensor's output rate.";
}

void MainWindow::loadDefinitions(const QString& path) {
  QString error;
  definitions_ = loadRegisterDefinitions(path, &error);
  if (!error.isEmpty()) setStatus("Register schema: " + error, true);
  refreshRegisterTable();
}

void MainWindow::refreshRegisterTable() {
  if (!register_tree_) return;
  const auto& t = theme();
  const QSignalBlocker blocker(register_tree_);
  const int selected_id = current_register_id_;
  register_tree_->clear();
  const QString filter = register_filter_ ? register_filter_->text().trimmed() : QString();
  QMap<QString, QTreeWidgetItem*> groups;
  QTreeWidgetItem* selected_item = nullptr;
  QColor staged_fill = t.change;
  staged_fill.setAlpha(t.dark ? 46 : 110);
  for (const auto& definition : definitions_) {
    const QString searchable = QString("%1 %2 %3 %4")
                                   .arg(definition.id).arg(definition.group, definition.name, definition.description);
    if (!filter.isEmpty() && !searchable.contains(filter, Qt::CaseInsensitive)) continue;
    auto* group = groups.value(definition.group, nullptr);
    if (!group) {
      group = new QTreeWidgetItem(register_tree_, {definition.group.toUpper()});
      group->setFirstColumnSpanned(true);
      group->setFlags(group->flags() & ~Qt::ItemIsSelectable);
      group->setFont(0, strongFont(12));
      group->setForeground(0, t.muted);
      groups.insert(definition.group, group);
    }
    QString state;
    if (device_values_.contains(definition.id)) state = "Read";
    if (staged_values_.contains(definition.id)) state = "Staged";
    auto* item = new QTreeWidgetItem(group, {QString("%1   %2").arg(definition.id, 3).arg(definition.name), state});
    item->setData(0, Qt::UserRole, definition.id);
    item->setToolTip(0, definition.writable ? definition.description
                                            : definition.description + "\n(read-only)");
    item->setFont(0, uiFont(14));
    if (!definition.writable) item->setForeground(0, t.muted);
    item->setForeground(1, t.muted);
    if (staged_values_.contains(definition.id)) {
      item->setBackground(0, staged_fill);
      item->setBackground(1, staged_fill);
      item->setFont(1, strongFont(13));
      item->setForeground(1, t.dark ? t.change : t.warn);
    } else {
      item->setFont(1, uiFont(13));
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
  registersChanged();
}

void MainWindow::registersChanged() {
  refreshStagedIndicators();
  refreshBandwidth();
  refreshCalibrationSummary();
  if (wizard_) wizard_->setRegisterContext(device_values_, staged_values_);
}

BandwidthEstimate MainWindow::bandwidth(bool with_staged) const {
  QMap<int, QString> registers = device_values_;
  if (with_staged)
    for (auto staged = staged_values_.cbegin(); staged != staged_values_.cend(); ++staged)
      registers[staged.key()] = staged.value();
  return estimateBandwidth(registers);
}

void MainWindow::refreshBandwidth() {
  if (!bandwidth_meter_) return;
  bool touches_output = false;
  for (int id : kOutputRegisters) touches_output |= staged_values_.contains(id);
  bandwidth_meter_->setEstimates(bandwidth(false), touches_output ? std::optional(bandwidth(true)) : std::nullopt);
  if (telemetry_chip_) refreshTelemetryChip();
}

void MainWindow::readOutputRegisters() {
  if (!connected_ || connection_mode_ == "demo") return;
  for (int id : kOutputRegisters) emit requestRead(id);
}

void MainWindow::scheduleAutoRead() {
  if (!auto_read_timer_ || !connected_ || connection_mode_ == "demo" || current_register_id_ < 0) return;
  // staged change is left alone; Read (F5) still fetches
  if (staged_values_.contains(current_register_id_)) return;
  for (int id : kNoAutoRead)
    if (id == current_register_id_) return;
  auto_read_timer_->start();
}

void MainWindow::refreshStagedIndicators() {
  if (!apply_button_) return;
  const int count = staged_values_.size();
  staged_chip_->setState(QString("%1 staged").arg(count), StatusChip::Level::Change);
  staged_chip_->setVisible(count > 0);
  apply_button_->setText(count ? QString("Apply %1").arg(count) : QString("Apply"));
  apply_button_->setEnabled(count > 0);
  const QString role = count ? "primary" : "";
  if (apply_button_->property("role").toString() != role) {
    apply_button_->setProperty("role", role);
    repolish(apply_button_);
  }
  if (auto* discard = findChild<QPushButton*>("discardButton")) discard->setEnabled(count > 0);
}

QStringList MainWindow::fieldNames(const RegisterDefinition& definition, int count) const {
  QStringList names;
  for (const auto& field : definition.fields) names << field.name;
  if (!names.isEmpty()) return names;
  if ((definition.id == 23 || definition.id == 25 || definition.id == 84) && count == 12)
    return {"C11", "C12", "C13", "C21", "C22", "C23", "C31", "C32", "C33", "Bias X", "Bias Y", "Bias Z"};
  if (count == 9 && definition.name.contains("Rotation"))
    return {"R11", "R12", "R13", "R21", "R22", "R23", "R31", "R32", "R33"};
  if (count == 4 && definition.name.contains("Quaternion", Qt::CaseInsensitive)) return {"X", "Y", "Z", "W"};
  if (count == 3) return {"X", "Y", "Z"};
  for (int index = 0; index < count; ++index) names << QString("Field %1").arg(index + 1);
  return names;
}

void MainWindow::selectRegister(QTreeWidgetItem* current, QTreeWidgetItem*) {
  if (!current || !current->parent()) return;
  current_register_id_ = current->data(0, Qt::UserRole).toInt();
  refreshRegisterInspector();
  scheduleAutoRead();
}

void MainWindow::refreshRegisterInspector() {
  if (!register_inspector_ || !raw_register_editor_) return;
  using Level = StatusChip::Level;
  const RegisterDefinition definition = definitionForId(current_register_id_);
  updating_register_editor_ = true;
  register_inspector_->setRowCount(0);
  if (definition.id < 0) {
    register_title_->setText("No register selected");
    register_description_->setText("Select a register to compare its device and staged values.");
    register_group_chip_->hide();
    register_access_chip_->hide();
    register_state_chip_->hide();
    raw_register_editor_->clear();
    raw_register_editor_->setEnabled(false);
    stage_raw_button_->setEnabled(false);
    updating_register_editor_ = false;
    return;
  }

  const QString device_text = device_values_.value(definition.id);
  const QString staged_text = staged_values_.value(definition.id, device_text);
  const QStringList device = device_text.split(';', Qt::KeepEmptyParts);
  const QStringList staged = staged_text.split(';', Qt::KeepEmptyParts);
  // optional trailing fields show only if present
  int count = std::max(device_text.isEmpty() ? 0 : device.size(), staged_text.isEmpty() ? 0 : staged.size());
  if (definition.value_count >= 0) count = std::max(count, definition.value_count);
  if (count == 0) count = std::max(1, definition.value_count < 0 ? 1 : static_cast<int>(definition.fields.size()));
  const QStringList names = fieldNames(definition, count);

  register_title_->setText(QString("%1  ·  %2").arg(definition.id).arg(definition.name));
  register_description_->setText(definition.description +
                                 (definition.firmware.isEmpty() ? QString() : "  ·  Firmware " + definition.firmware));
  register_group_chip_->setState(definition.group, Level::Neutral);
  register_access_chip_->setState(definition.writable ? (definition.persistent ? "Writable · flash" : "Writable")
                                                      : "Read-only",
                                  definition.writable ? Level::Info : Level::Neutral);
  register_state_chip_->setState(staged_values_.contains(definition.id) ? "Staged, sensor unchanged"
                                 : device_values_.contains(definition.id) ? "Device value"
                                                                          : "Not read",
                                 staged_values_.contains(definition.id) ? Level::Change
                                 : device_values_.contains(definition.id) ? Level::Ok
                                                                          : Level::Neutral);
  register_group_chip_->show();
  register_access_chip_->show();
  register_state_chip_->show();
  const bool binary = definition.editor == "binary_output";
  inspector_stack_->setCurrentIndex(binary ? 1 : 0);
  if (binary) {
    binary_editor_->setValues(definition.id, device_text, staged_text, device_values_);
    binary_editor_->setEditable(definition.writable);
  }

  QColor changed = theme().change;
  changed.setAlpha(theme().dark ? 60 : 130);
  register_inspector_->setRowCount(count);
  for (int row = 0; row < count; ++row) {
    QString field_name = names.value(row, QString("Field %1").arg(row + 1));
    QString units = definition.units;
    if (row < definition.fields.size() && !definition.fields[row].units.isEmpty())
      units = definition.fields[row].units;
    auto* name_item = new QTableWidgetItem(field_name);
    auto* device_item = new QTableWidgetItem(device.value(row));
    auto* staged_item = new QTableWidgetItem(staged.value(row));
    auto* units_item = new QTableWidgetItem(units);
    device_item->setFont(figuresFont(15));
    staged_item->setFont(figuresFont(15));
    units_item->setForeground(theme().muted);
    if (row < definition.fields.size() && !definition.fields[row].enum_values.isEmpty()) {
      QStringList choices;
      const auto& values = definition.fields[row].enum_values;
      for (auto value = values.cbegin(); value != values.cend(); ++value)
        choices << value.key() + " = " + value.value();
      name_item->setToolTip(choices.join("\n"));
      const QString device_choice = values.value(device.value(row));
      if (!device_choice.isEmpty()) device_item->setText(device_choice + "  [" + device.value(row) + "]");
    }
    name_item->setFlags(name_item->flags() & ~Qt::ItemIsEditable);
    device_item->setFlags(device_item->flags() & ~Qt::ItemIsEditable);
    units_item->setFlags(units_item->flags() & ~Qt::ItemIsEditable);
    if (!definition.writable) staged_item->setFlags(staged_item->flags() & ~Qt::ItemIsEditable);
    if (device.value(row) != staged.value(row)) staged_item->setBackground(changed);
    register_inspector_->setItem(row, 0, name_item);
    register_inspector_->setItem(row, 1, device_item);
    register_inspector_->setItem(row, 2, staged_item);
    register_inspector_->setItem(row, 3, units_item);
    if (definition.writable && row < definition.fields.size()) {
      const auto& field = definition.fields[row];
      if (!field.enum_values.isEmpty()) {
        auto* editor = new QComboBox;
        QList<std::pair<double, QString>> keys;
        for (auto value = field.enum_values.cbegin(); value != field.enum_values.cend(); ++value)
          keys << std::make_pair(value.key().toDouble(), value.key());
        std::sort(keys.begin(), keys.end());
        for (const auto& key : keys) editor->addItem(field.enum_values.value(key.second) + "  [" + key.second + "]",
                                                     key.second);
        // unlisted value stays selectable
        int selected = -1;
        for (int i = 0; i < editor->count(); ++i)
          if (registerValuesMatch(editor->itemData(i).toString(), staged.value(row))) selected = i;
        if (selected < 0) {
          editor->addItem(staged.value(row).isEmpty() ? QString("(not read)")
                                                      : staged.value(row) + "  (not in the manual's list)",
                          staged.value(row));
          selected = editor->count() - 1;
        }
        editor->setCurrentIndex(selected);
        register_inspector_->setCellWidget(row, 2, editor);
        connect(editor, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int) { stageInspectorValues(); });
      } else if (field.has_range) {
        auto* editor = new QDoubleSpinBox;
        editor->setRange(field.minimum, field.maximum);
        editor->setDecimals(7);
        editor->setValue(staged.value(row).toDouble());
        editor->setFont(figuresFont(15));
        register_inspector_->setCellWidget(row, 2, editor);
        connect(editor, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
                [this](double) { stageInspectorValues(); });
      }
    }
  }
  raw_register_editor_->setText(staged_text);
  raw_register_editor_->setEnabled(definition.writable);
  stage_raw_button_->setEnabled(definition.writable);
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
  if (registerValuesMatch(serialized, device_values_.value(current_register_id_)))
    staged_values_.remove(current_register_id_);
  else
    staged_values_[current_register_id_] = serialized;
  raw_register_editor_->setText(serialized);
  register_state_chip_->setState("Staged, sensor unchanged", StatusChip::Level::Change);
  registersChanged();
  setStatus(definition.name + " staged; sensor unchanged");
}

void MainWindow::setStatus(const QString& text, bool error) {
  if (!status_) return;
  status_->setText(text);
  const QString tone = error ? "error" : "";
  if (status_->property("tone").toString() != tone) {
    status_->setProperty("tone", tone);
    repolish(status_);
  }
}

bool MainWindow::confirm(const QString& title, const QString& text, bool warning) {
  return (warning ? QMessageBox::warning(this, title, text, QMessageBox::Yes | QMessageBox::Cancel)
                  : QMessageBox::question(this, title, text)) == QMessageBox::Yes;
}

bool MainWindow::needsMaintenance(const QString& action, std::function<void()> retry) {
  if (connection_mode_ != "ros" || maintenance_active_) return false;
  askForMaintenance(action, std::move(retry));
  return true;
}

void MainWindow::askForMaintenance(const QString& action, std::function<void()> retry) {
  QMessageBox box(QMessageBox::Warning, "Maintenance mode required",
                  action + " needs maintenance mode while connected to the robot's driver.", QMessageBox::Cancel,
                  this);
  box.setInformativeText("Maintenance mode lets this tool change driver-critical registers, reset the sensor "
                         "and save to flash. These can briefly interrupt IMU data on the robot; the driver keeps "
                         "running. Leave maintenance mode when you are done.");
  auto* enter = box.addButton(retry ? "Enter maintenance and continue" : "Enter maintenance",
                              QMessageBox::AcceptRole);
  box.setDefaultButton(enter);
  box.exec();
  if (box.clickedButton() != enter) {
    setStatus(action + " cancelled: maintenance mode is off", true);
    return;
  }
  after_maintenance_ = std::move(retry);
  emit requestOpenMaintenance();
}

void MainWindow::resetSensor() {
  if (needsMaintenance("Resetting the sensor", [this] { resetSensor(); })) return;
  if (confirm("Reset sensor", "Reset the sensor? The connection will be interrupted.", true)) emit requestReset();
}

RegisterDefinition MainWindow::definitionForId(int id) const {
  for (const auto& definition : definitions_)
    if (definition.id == id) return definition;
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
  if (wizard_ && wizard_->isVisible()) wizard_->addSample(sample);
}

void MainWindow::onRegisterRead(int id, const QString& values) {
  device_values_[id] = values;
  if (staged_values_.contains(id) && registerValuesMatch(staged_values_.value(id), values)) staged_values_.remove(id);
  refreshRegisterTable();
  if (wizard_) wizard_->setRegistersMissing(missingCalibrationRegisters());
}

void MainWindow::readSelectedRegister() {
  if (current_register_id_ < 0) {
    setStatus("Select a register to read", true);
    return;
  }
  emit requestRead(current_register_id_);
}

void MainWindow::applyStagedRegisters() {
  if (staged_values_.isEmpty()) {
    setStatus("No staged changes");
    return;
  }
  const QMap<int, QString> pending = staged_values_;
  for (auto iterator = pending.cbegin(); iterator != pending.cend(); ++iterator) {
    QString error;
    const auto definition = definitionForId(iterator.key());
    if (definition.id < 0) {
      setStatus(QString("Register %1 is unsupported by this schema").arg(iterator.key()), true);
      return;
    }
    if (!validateRegisterValue(definition, iterator.value(), &error)) {
      setStatus(definition.name + ": " + error, true);
      return;
    }
    if (definition.driver_critical &&
        needsMaintenance("Writing " + definition.name, [this] { applyStagedRegisters(); }))
      return;
  }
  // output config has to fit the serial line
  bool touches_output = false;
  for (int id : kOutputRegisters) touches_output |= pending.contains(id);
  if (touches_output && connection_mode_ != "demo") {
    const auto projected = bandwidth(true);
    if (!projected.known) {
      readOutputRegisters();
      setStatus("Reading registers " + projected.missing + " to check the line load first; apply again after", true);
      return;
    }
    if (projected.exceeds()) {
      QMessageBox::warning(this, "Output exceeds the baud rate",
                           QString("With these changes the sensor's output needs %1% of the %2 baud line:\n\n%3"
                                   "\n\nLower an output's rate, drop fields, or raise the baud rate first.")
                               .arg(100 * projected.fraction(), 0, 'f', 0).arg(projected.baud)
                               .arg(projected.parts.join("\n")));
      setStatus("Not applied: output would exceed the baud rate", true);
      return;
    }
  }
  for (auto iterator = pending.cbegin(); iterator != pending.cend(); ++iterator) {
    const auto definition = definitionForId(iterator.key());
    if (definition.dangerous &&
        !confirm("Connection-affecting register", definition.name + " can interrupt the connection. Apply it?"))
      return;
  }
  for (auto iterator = pending.cbegin(); iterator != pending.cend(); ++iterator)
    emit requestWrite(iterator.key(), iterator.value());
}

void MainWindow::discardStagedRegisters() {
  staged_values_.clear();
  refreshRegisterTable();
  setStatus("Staged changes discarded");
}

void MainWindow::importConfiguration() {
  const QString path =
      QFileDialog::getOpenFileName(this, "Import Control Center configuration", {}, "VectorNav XML (*.xml)");
  if (path.isEmpty()) return;
  QString error;
  const auto imported = loadControlCenterXml(path, &error);
  if (!error.isEmpty()) {
    setStatus(error, true);
    return;
  }
  if (!confirm("Stage configuration",
               QString("Stage %1 register entries? Nothing will be written yet.").arg(imported.registers.size())))
    return;
  configuration_metadata_ = imported;
  for (const auto& reg : imported.registers) staged_values_[reg.id] = reg.values;
  refreshRegisterTable();
}

void MainWindow::exportConfiguration() {
  const QString path =
      QFileDialog::getSaveFileName(this, "Export Control Center configuration", {}, "VectorNav XML (*.xml)");
  if (path.isEmpty()) return;
  SensorConfiguration output = configuration_metadata_;
  output.registers.clear();
  for (const auto& definition : definitions_) {
    if (device_values_.contains(definition.id) || staged_values_.contains(definition.id))
      output.registers.push_back({definition.id, definition.name,
                                  staged_values_.value(definition.id, device_values_.value(definition.id)), true});
  }
  for (const auto& unknown : configuration_metadata_.registers)
    if (definitionForId(unknown.id).id < 0) output.registers.push_back(unknown);
  QString error;
  if (!saveControlCenterXml(path, output, &error))
    setStatus(error, true);
  else
    setStatus("Configuration exported");
}

void MainWindow::savePersistently() {
  if (!staged_values_.isEmpty()) {
    QMessageBox::warning(this, "Unsaved staged values",
                         "Apply and verify staged changes before saving persistent sensor state.");
    return;
  }
  if (needsMaintenance("Saving to flash", [this] { savePersistently(); })) return;
  const auto load = bandwidth(false);
  if (connection_mode_ != "demo" && !load.known) {
    readOutputRegisters();
    setStatus("Reading registers " + load.missing + " to check the line load before saving; save again after", true);
    return;
  }
  if (load.exceeds()) {
    QMessageBox::warning(this, "Output exceeds the baud rate",
                         QString("The sensor's output needs %1% of its %2 baud line, so it is already dropping "
                                 "data. Fix the output configuration before saving it to flash.")
                             .arg(100 * load.fraction(), 0, 'f', 0).arg(load.baud));
    return;
  }
  if (!confirm("Save, reset, and verify",
               "Persist the complete active configuration, reset the sensor, reconnect the driver, "
               "and verify all values read by this dashboard? IMU output will be stale briefly."))
    return;
  QJsonObject expected;
  for (const auto& definition : definitions_) {
    if (!definition.writable || !definition.persistent || !device_values_.contains(definition.id)) continue;
    QJsonArray fields;
    for (const auto& value : device_values_.value(definition.id).split(';')) fields.push_back(value);
    expected[QString::number(definition.id)] = fields;
  }
  emit requestSaveSettings(QString::fromUtf8(QJsonDocument(expected).toJson(QJsonDocument::Compact)));
}

void MainWindow::openCalibrationWizard() {
  if (!wizard_) {
    wizard_ = new CalibrationWizard(&calibration_, policy_, this);
    connect(wizard_, &CalibrationWizard::captureRequested, this, &MainWindow::requestCalibrationCapture);
    connect(wizard_, &CalibrationWizard::registersRequested, this, [this] {
      if (!connected_) return;
      for (int id : kCalibrationRegisters) emit requestRead(id);
    });
    connect(wizard_, &CalibrationWizard::stageRequested, this, &MainWindow::stageCalibration);
    connect(wizard_, &CalibrationWizard::exportRequested, this, &MainWindow::exportCalibrationReport);
    connect(wizard_, &CalibrationWizard::sessionChanged, this, &MainWindow::refreshCalibrationSummary);
  }
  wizard_->setConnected(connected_);
  wizard_->setRegistersMissing(missingCalibrationRegisters());
  wizard_->setRegisterContext(device_values_, staged_values_);
  wizard_->show();
  wizard_->raise();
  wizard_->activateWindow();
}

QString MainWindow::missingCalibrationRegisters() const {
  QStringList missing;
  for (int id : kCalibrationRegisters)
    if (!device_values_.contains(id)) missing << QString::number(id);
  return missing.join(", ");
}

void MainWindow::refreshCalibrationSummary() {
  if (!calibration_summary_) return;
  using Level = StatusChip::Level;
  std::vector<MetricRow> rows;
  int stageable = 0;
  QStringList notes;
  for (int index = 0; index < kCalibrationStepCount; ++index) {
    const auto step = static_cast<CalibrationStep>(index);
    const int id = compensationRegister(step);
    QString verdict = "Not run";
    Level level = Level::Neutral;
    if (calibration_.evaluated[index]) {
      const bool passed = calibration_.quality(step).passed;
      verdict = passed ? "Passed" : "Failed";
      level = passed ? Level::Ok : Level::Error;
      // checked against the registers every time
      const auto delivery = deliveryState(calibration_, step, device_values_, staged_values_);
      if (passed && delivery == Delivery::Staged) verdict = "Staged", level = Level::Change;
      if (passed && delivery == Delivery::Applied) verdict = "On sensor", level = Level::Info;
      if (passed && id >= 0 && delivery == Delivery::None) {
        ++stageable;
        if (!calibration_.delivered[index].isEmpty())
          notes << QString("%1: no longer staged or on the sensor.").arg(calibrationStepName(step));
      }
    }
    rows.push_back({calibrationStepName(step), calibrationHeadline(calibration_, step),
                    id >= 0 ? QString::number(id) : QString("none"), level, verdict});
  }
  calibration_summary_->setRows(rows);
  stage_latest_button_->setEnabled(stageable > 0);
  stage_latest_button_->setText(stageable ? QString("Stage latest results (%1)").arg(stageable)
                                          : QString("Stage latest results"));
  if (stageable) notes << "Stage latest results copies them into the register workspace again.";
  calibration_note_->setText(notes.join(" "));
  calibration_note_->setVisible(!notes.isEmpty());
}

void MainWindow::stageCalibration(CalibrationStep step) {
  auto finish = [&](bool staged, const QString& message) {
    setStatus(message, !staged);
    if (wizard_) wizard_->setStageOutcome(step, staged, message);
    refreshCalibrationSummary();
  };
  const auto proposal = proposeCompensation(calibration_, step, device_values_);
  if (!proposal.error.isEmpty()) {
    for (int id : proposal.unread) emit requestRead(id);
    finish(false, proposal.error);
    return;
  }
  staged_values_[proposal.register_id] = proposal.values;
  calibration_.delivered[static_cast<int>(step)] = proposal.values;
  refreshRegisterTable();
  finish(true, QString("Register %1 staged; review it in the register list, then Apply").arg(proposal.register_id));
}

void MainWindow::stageLatestCalibration() {
  for (int index = 0; index < kCalibrationStepCount; ++index) {
    const auto step = static_cast<CalibrationStep>(index);
    if (compensationRegister(step) < 0 || !calibration_.evaluated[index] || !calibration_.quality(step).passed)
      continue;
    if (deliveryState(calibration_, step, device_values_, staged_values_) == Delivery::None) stageCalibration(step);
  }
}

void MainWindow::exportProtocolLog() {
  const QString stamp = QDateTime::currentDateTime().toString("yyyy-MM-dd_HHmmss");
  const QString path = QFileDialog::getSaveFileName(this, "Export protocol log",
                                                    QDir::home().filePath("vn_protocol_" + stamp + ".txt"),
                                                    "Text (*.txt *.log)");
  if (path.isEmpty()) return;
  QFile file(path);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
    setStatus(file.errorString(), true);
    return;
  }
  const QString header = QString("# VectorNav Workshop protocol log, exported %1\n# Connection: %2 %3\n")
                             .arg(QDateTime::currentDateTime().toString(Qt::ISODate), connection_mode_,
                                  connected_target_.isEmpty() ? QString("(disconnected)") : connected_target_);
  file.write(header.toUtf8());
  file.write(traffic_log_->toPlainText().toUtf8());
  file.write("\n");
  setStatus(QString("Protocol log exported to %1").arg(path));
}

void MainWindow::exportCalibrationReport() {
  const QString path = QFileDialog::getSaveFileName(this, "Export calibration report", {}, "JSON (*.json)");
  if (path.isEmpty()) return;
  const QJsonObject report = calibrationReport(calibration_, policy_);
  refreshCalibrationSummary();
  QFile file(path);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
    setStatus(file.errorString(), true);
    return;
  }
  file.write(QJsonDocument(report).toJson(QJsonDocument::Indented));
  setStatus("Calibration report exported");
}

void MainWindow::exportTelemetry() {
  const QString path = QFileDialog::getSaveFileName(this, "Export telemetry", {}, "CSV (*.csv)");
  if (path.isEmpty()) return;
  QFile file(path);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
    setStatus(file.errorString(), true);
    return;
  }
  file.write("time_s,qx,qy,qz,qw,mx,my,mz,ax,ay,az,gx,gy,gz,temperature_c,pressure_kpa\n");
  for (const auto& s : recorded_telemetry_) {
    const QString line = QString("%1,%2,%3,%4,%5,%6,%7,%8,%9,%10,%11,%12,%13,%14,%15,%16\n")
        .arg(s.time_s, 0, 'g', 16).arg(s.qx, 0, 'g', 12).arg(s.qy, 0, 'g', 12).arg(s.qz, 0, 'g', 12)
        .arg(s.qw, 0, 'g', 12).arg(s.mx, 0, 'g', 12).arg(s.my, 0, 'g', 12).arg(s.mz, 0, 'g', 12)
        .arg(s.ax, 0, 'g', 12).arg(s.ay, 0, 'g', 12).arg(s.az, 0, 'g', 12).arg(s.gx, 0, 'g', 12)
        .arg(s.gy, 0, 'g', 12).arg(s.gz, 0, 'g', 12).arg(s.temperature_c, 0, 'g', 12)
        .arg(s.pressure_kpa, 0, 'g', 12);
    file.write(line.toUtf8());
  }
  setStatus(QString("Exported %1 telemetry samples").arg(recorded_telemetry_.size()));
}

void MainWindow::loadReplay() {
  const QString path = QFileDialog::getOpenFileName(this, "Replay telemetry", {}, "CSV (*.csv)");
  if (path.isEmpty()) return;
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) {
    setStatus(file.errorString(), true);
    return;
  }
  replay_telemetry_.clear();
  file.readLine();
  while (!file.atEnd()) {
    const auto fields = QString::fromUtf8(file.readLine()).trimmed().split(',');
    if (fields.size() != 16) continue;
    Telemetry s;
    bool ok = true;
    double* values[] = {&s.time_s, &s.qx, &s.qy, &s.qz, &s.qw, &s.mx, &s.my, &s.mz,
                        &s.ax, &s.ay, &s.az, &s.gx, &s.gy, &s.gz, &s.temperature_c, &s.pressure_kpa};
    for (int i = 0; i < 16; ++i) {
      bool field_ok = false;
      *values[i] = fields[i].toDouble(&field_ok);
      ok &= field_ok;
    }
    if (ok) replay_telemetry_.push_back(s);
  }
  if (replay_telemetry_.empty()) {
    setStatus("Replay file has no valid telemetry rows", true);
    return;
  }
  if (!replay_timer_) {
    replay_timer_ = new QTimer(this);
    replay_timer_->setInterval(50);
    connect(replay_timer_, &QTimer::timeout, this, [this] {
      if (replay_telemetry_.empty()) {
        replay_timer_->stop();
        setStatus("Replay complete");
        return;
      }
      const bool was_recording = recording_;
      recording_ = false;
      onTelemetry(replay_telemetry_.front());
      recording_ = was_recording;
      replay_telemetry_.pop_front();
    });
  }
  plot_->clear();
  replay_timer_->start();
  setStatus("Replaying telemetry through the monitor");
}

}  // namespace riptide_imu
