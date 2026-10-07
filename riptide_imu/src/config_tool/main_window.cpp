#include "riptide_imu/config_tool/main_window.hpp"

#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
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
#include <QMessageBox>
#include <QPainter>
#include <QPainterPath>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSettings>
#include <QSplitter>
#include <QStackedWidget>
#include <QStatusBar>
#include <QTableWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <cmath>

namespace riptide_imu {
namespace {

QLabel* title(const QString& text) {
  auto* label = new QLabel(text);
  label->setObjectName("pageTitle");
  return label;
}

QString vectorText(const Eigen::Vector3d& value, int precision = 7) {
  return QString("[%1, %2, %3]")
      .arg(value.x(), 0, 'g', precision).arg(value.y(), 0, 'g', precision)
      .arg(value.z(), 0, 'g', precision);
}

}  // namespace

AttitudeWidget::AttitudeWidget(QWidget* parent) : QWidget(parent) {
  setMinimumSize(320, 260);
}

void AttitudeWidget::setQuaternion(double x, double y, double z, double w) {
  Eigen::Quaterniond q(w, x, y, z);
  if (q.norm() > 1e-9 && q.coeffs().allFinite()) orientation_ = q.normalized();
  update();
}

void AttitudeWidget::paintEvent(QPaintEvent*) {
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.fillRect(rect(), palette().window());
  painter.translate(width() / 2.0, height() / 2.0);
  const Eigen::Matrix3d rotation = orientation_.toRotationMatrix();
  auto project = [&](const Eigen::Vector3d& value) {
    const Eigen::Vector3d r = rotation * value;
    return QPointF(82.0 * (r.x() + 0.35 * r.z()),
                   -82.0 * (r.y() - 0.2 * r.z()));
  };
  const Eigen::Vector3d vertices[] = {
      {-1, -0.55, -0.25}, {1, -0.55, -0.25}, {1, 0.55, -0.25}, {-1, 0.55, -0.25},
      {-1, -0.55, 0.25}, {1, -0.55, 0.25}, {1, 0.55, 0.25}, {-1, 0.55, 0.25}};
  const int edges[][2] = {{0,1},{1,2},{2,3},{3,0},{4,5},{5,6},{6,7},{7,4},{0,4},{1,5},{2,6},{3,7}};
  painter.setPen(QPen(QColor("#8aa4c5"), 3));
  for (const auto& edge : edges) painter.drawLine(project(vertices[edge[0]]), project(vertices[edge[1]]));
  const QColor colors[] = {QColor("#ff6262"), QColor("#58d68d"), QColor("#5dade2")};
  for (int axis = 0; axis < 3; ++axis) {
    painter.setPen(QPen(colors[axis], 4));
    painter.drawLine(QPointF(0, 0), project(Eigen::Vector3d::Unit(axis) * 1.4));
  }
  painter.setPen(palette().text().color());
  painter.drawText(QRectF(-width()/2.0 + 12, -height()/2.0 + 8, width()-24, 30),
                   "Orientation only — position is not inferred");
}

PlotWidget::PlotWidget(QWidget* parent) : QWidget(parent) { setMinimumHeight(230); }

void PlotWidget::append(const Telemetry& sample) {
  samples_.push_back(sample);
  while (samples_.size() > 300) samples_.pop_front();
  update();
}

void PlotWidget::paintEvent(QPaintEvent*) {
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.fillRect(rect(), palette().base());
  const QRectF area = rect().adjusted(48, 22, -15, -28);
  painter.setPen(QPen(palette().mid().color(), 1));
  for (int i = 0; i <= 4; ++i)
    painter.drawLine(area.left(), area.top() + i * area.height()/4.0,
                     area.right(), area.top() + i * area.height()/4.0);
  if (samples_.size() < 2) return;
  double maximum = 0.1;
  for (const auto& sample : samples_)
    maximum = std::max({maximum, std::abs(sample.gx), std::abs(sample.gy), std::abs(sample.gz)});
  const QColor colors[] = {QColor("#ff6262"), QColor("#58d68d"), QColor("#5dade2")};
  for (int axis = 0; axis < 3; ++axis) {
    QPainterPath path;
    for (std::size_t i = 0; i < samples_.size(); ++i) {
      const auto& sample = samples_[i];
      const double value = axis == 0 ? sample.gx : axis == 1 ? sample.gy : sample.gz;
      const QPointF point(area.left() + i * area.width()/(samples_.size()-1),
                          area.center().y() - value * area.height()/(2.0*maximum));
      i == 0 ? path.moveTo(point) : path.lineTo(point);
    }
    painter.setPen(QPen(colors[axis], 2)); painter.drawPath(path);
  }
  painter.setPen(palette().text().color());
  painter.drawText(8, 18, "Angular rate (rad/s)");
}

MainWindow::MainWindow(QString imu_port, int baud, QString config_file, bool demo,
                       QString share_directory, QWidget* parent)
    : QMainWindow(parent), share_directory_(std::move(share_directory)) {
  qRegisterMetaType<Telemetry>();
  setWindowTitle("VectorNav Workshop");
  resize(1280, 820);
  auto* central = new QWidget;
  auto* root = new QHBoxLayout(central); root->setContentsMargins(0, 0, 0, 0);
  navigation_ = new QListWidget;
  navigation_->addItems({"Monitor", "Registers", "Calibration", "Diagnostics"});
  navigation_->setFixedWidth(190); navigation_->setCurrentRow(0);
  pages_ = new QStackedWidget;
  pages_->addWidget(buildMonitorPage()); pages_->addWidget(buildRegistersPage());
  pages_->addWidget(buildCalibrationPage()); pages_->addWidget(buildDiagnosticsPage());
  auto* workspace = new QWidget;
  auto* workspace_layout = new QVBoxLayout(workspace);
  workspace_layout->setContentsMargins(12, 8, 12, 8);
  workspace_layout->addWidget(buildConnectionBar(imu_port, baud));
  workspace_layout->addWidget(pages_, 1);
  root->addWidget(navigation_); root->addWidget(workspace, 1); setCentralWidget(central);
  connect(navigation_, &QListWidget::currentRowChanged, pages_, &QStackedWidget::setCurrentIndex);

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

  transport_ = new SensorTransport;
  transport_->moveToThread(&sensor_thread_);
  connect(&sensor_thread_, &QThread::finished, transport_, &QObject::deleteLater);
  connect(this, &MainWindow::requestConnect, transport_, &SensorTransport::connectSensor);
  connect(this, &MainWindow::requestDisconnect, transport_, &SensorTransport::disconnectSensor);
  connect(this, &MainWindow::requestDemo, transport_, &SensorTransport::setDemoMode);
  connect(this, &MainWindow::requestRead, transport_, &SensorTransport::readRegister);
  connect(this, &MainWindow::requestWrite, transport_, &SensorTransport::writeRegister);
  connect(this, &MainWindow::requestSaveSettings, transport_, &SensorTransport::writeSettings);
  connect(this, &MainWindow::requestTare, transport_, &SensorTransport::tare);
  connect(this, &MainWindow::requestReset, transport_, &SensorTransport::reset);
  connect(transport_, &SensorTransport::telemetryReceived, this, &MainWindow::onTelemetry);
  connect(transport_, &SensorTransport::registerRead, this, &MainWindow::onRegisterRead);
  connect(transport_, &SensorTransport::writeVerified, this, &MainWindow::onRegisterRead);
  connect(transport_, &SensorTransport::connectionChanged, this,
          [this](bool ok, const QString& detail) {
            connected_ = ok;
            if (connection_button_) {
              connection_button_->setEnabled(true);
              connection_button_->setText(ok ? "Disconnect" : "Connect");
            }
            if (port_selector_) port_selector_->setEnabled(!ok);
            if (baud_selector_) baud_selector_->setEnabled(!ok);
            setStatus(detail, !ok && detail != "Disconnected");
          });
  connect(transport_, &SensorTransport::operationFailed, this,
          [this](const QString& op, const QString& detail) { setStatus(op + ": " + detail, true); });
  connect(transport_, &SensorTransport::traffic, this,
          [this](const QString& direction, const QString& packet) {
            if (traffic_log_) traffic_log_->appendPlainText(direction + "  " + packet);
          });
  sensor_thread_.start();
  emit requestDemo(demo);
  connection_button_->setEnabled(false);
  setStatus(demo ? "Starting demo data…" : QString("Connecting to %1…").arg(imu_port));
  emit requestConnect(imu_port, baud);
  if (!config_file.isEmpty()) {
    QString error; configuration_metadata_ = loadControlCenterXml(config_file, &error);
    for (const auto& reg : configuration_metadata_.registers) staged_values_[reg.id] = reg.values;
    refreshRegisterTable();
    if (!error.isEmpty()) setStatus("Configuration import: " + error, true);
  }
}

QWidget* MainWindow::buildConnectionBar(const QString& initial_port, int initial_baud) {
  auto* bar = new QWidget;
  bar->setObjectName("connectionBar");
  auto* layout = new QHBoxLayout(bar);
  layout->setContentsMargins(10, 6, 10, 6);
  layout->addWidget(new QLabel("Sensor port"));

  port_selector_ = new QComboBox;
  port_selector_->setObjectName("sensorPortSelector");
  port_selector_->setAccessibleName("Sensor serial port");
  port_selector_->setEditable(true);
  port_selector_->setInsertPolicy(QComboBox::NoInsert);
  port_selector_->setMinimumContentsLength(24);
  port_selector_->setToolTip(
      "The imu_port launch parameter supplies the initial value. Select a detected device or enter any serial path.");
  if (!initial_port.trimmed().isEmpty()) port_selector_->addItem(initial_port.trimmed());
  refreshSerialPorts();
  port_selector_->setCurrentText(initial_port);
  layout->addWidget(port_selector_, 1);

  auto* refresh = new QPushButton("Refresh");
  refresh->setObjectName("refreshSerialPortsButton");
  refresh->setToolTip("Rescan common Linux serial device paths");
  connect(refresh, &QPushButton::clicked, this, &MainWindow::refreshSerialPorts);
  layout->addWidget(refresh);

  layout->addWidget(new QLabel("Baud"));
  baud_selector_ = new QComboBox;
  baud_selector_->setObjectName("sensorBaudSelector");
  baud_selector_->setAccessibleName("Sensor baud rate");
  baud_selector_->setEditable(true);
  baud_selector_->addItems({"9600", "19200", "38400", "57600", "115200", "128000",
                            "230400", "460800", "921600"});
  baud_selector_->setCurrentText(QString::number(initial_baud));
  layout->addWidget(baud_selector_);

  connection_button_ = new QPushButton("Connect");
  connection_button_->setAccessibleName("Connect or disconnect sensor");
  connection_button_->setObjectName("primaryButton");
  connect(connection_button_, &QPushButton::clicked, this, &MainWindow::toggleConnection);
  layout->addWidget(connection_button_);
  return bar;
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

  const QString port = port_selector_->currentText().trimmed();
  bool valid_baud = false;
  const int baud = baud_selector_->currentText().trimmed().toInt(&valid_baud);
  if (port.isEmpty()) {
    setStatus("Select or enter a sensor port", true);
    port_selector_->setFocus();
    return;
  }
  if (!valid_baud || baud <= 0) {
    setStatus("Enter a valid positive baud rate", true);
    baud_selector_->setFocus();
    return;
  }
  connection_button_->setEnabled(false);
  setStatus(QString("Connecting to %1…").arg(port));
  emit requestConnect(port, baud);
}

MainWindow::~MainWindow() {
  QMetaObject::invokeMethod(transport_, "disconnectSensor", Qt::BlockingQueuedConnection);
  sensor_thread_.quit();
  sensor_thread_.wait(3000);
}

QWidget* MainWindow::buildMonitorPage() {
  auto* page = new QWidget; auto* layout = new QVBoxLayout(page); layout->addWidget(title("Live monitor"));
  auto* controls = new QHBoxLayout;
  auto* record = new QPushButton("Start recording");
  auto* pause = new QPushButton("Pause display");
  auto* export_button = new QPushButton("Export CSV…");
  auto* replay = new QPushButton("Replay CSV…");
  controls->addWidget(record); controls->addWidget(pause); controls->addWidget(export_button);
  controls->addWidget(replay); controls->addStretch(); layout->addLayout(controls);
  connect(record, &QPushButton::clicked, this, [this, record] {
    recording_ = !recording_;
    record->setText(recording_ ? "Stop recording" : "Start recording");
    setStatus(recording_ ? "Recording telemetry in memory" : "Recording stopped");
  });
  connect(pause, &QPushButton::clicked, this, [this, pause] {
    monitor_paused_ = !monitor_paused_;
    pause->setText(monitor_paused_ ? "Resume display" : "Pause display");
  });
  connect(export_button, &QPushButton::clicked, this, &MainWindow::exportTelemetry);
  connect(replay, &QPushButton::clicked, this, &MainWindow::loadReplay);
  auto* top = new QHBoxLayout; attitude_ = new AttitudeWidget; numeric_data_ = new QLabel("Waiting for data…");
  numeric_data_->setTextInteractionFlags(Qt::TextSelectableByMouse); numeric_data_->setMinimumWidth(340);
  top->addWidget(attitude_, 2); top->addWidget(numeric_data_, 1); layout->addLayout(top);
  plot_ = new PlotWidget; layout->addWidget(plot_, 1); return page;
}

QWidget* MainWindow::buildRegistersPage() {
  auto* page = new QWidget; auto* layout = new QVBoxLayout(page); layout->addWidget(title("Registers"));
  register_filter_ = new QLineEdit; register_filter_->setPlaceholderText("Search ID, name, group, or description…");
  layout->addWidget(register_filter_); register_table_ = new QTableWidget(0, 6);
  register_table_->setHorizontalHeaderLabels({"ID", "Group", "Register", "Device", "Staged", "Description"});
  register_table_->horizontalHeader()->setSectionResizeMode(5, QHeaderView::Stretch);
  register_table_->setSelectionBehavior(QAbstractItemView::SelectRows); layout->addWidget(register_table_, 1);
  auto* buttons = new QHBoxLayout;
  for (auto item : std::initializer_list<std::pair<QString, void (MainWindow::*)()>>{
      {"Read selected", &MainWindow::readSelectedRegister}, {"Apply staged", &MainWindow::applyStagedRegisters},
      {"Discard staged", &MainWindow::discardStagedRegisters}, {"Import XML", &MainWindow::importConfiguration},
      {"Export XML", &MainWindow::exportConfiguration}, {"Save persistently…", &MainWindow::savePersistently}}) {
    auto* button = new QPushButton(item.first); connect(button, &QPushButton::clicked, this, item.second); buttons->addWidget(button);
  }
  layout->addLayout(buttons);
  connect(register_filter_, &QLineEdit::textChanged, this, [this] { refreshRegisterTable(); });
  connect(register_table_, &QTableWidget::cellChanged, this, [this](int row, int column) {
    if (column != 4) return;
    staged_values_[register_table_->item(row, 0)->text().toInt()] =
        register_table_->item(row, 4)->text();
  });
  return page;
}

QWidget* MainWindow::buildCalibrationPage() {
  auto* page = new QWidget; auto* layout = new QVBoxLayout(page); layout->addWidget(title("Guided calibration"));
  auto* explanation = new QLabel(
      "Steps: 1 Gyro stationary poses · 2 Accelerometer varied stationary poses · 3 Magnetometer horizontal circle · "
      "4 Gravity report. Each step is independent. Skipped steps preserve their registers. Collection never writes the sensor.");
  explanation->setWordWrap(true); layout->addWidget(explanation);
  auto* row = new QHBoxLayout; auto* selector = new QComboBox;
  selector->addItems({"Gyro bias", "Accelerometer bias + magnitude", "Planar magnetometer", "Gravity/tilt report"});
  connect(selector, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int index) { calibration_step_ = index; });
  capture_button_ = new QPushButton("Start capture"); auto* evaluate = new QPushButton("Evaluate quality");
  auto* stage = new QPushButton("Stage passed result"); auto* clear = new QPushButton("Clear selected samples");
  row->addWidget(selector); row->addWidget(capture_button_); row->addWidget(evaluate); row->addWidget(stage); row->addWidget(clear); layout->addLayout(row);
  calibration_result_ = new QPlainTextEdit; calibration_result_->setReadOnly(true); layout->addWidget(calibration_result_, 1);
  auto* export_button = new QPushButton("Export machine-readable report…"); layout->addWidget(export_button);
  connect(capture_button_, &QPushButton::clicked, this, &MainWindow::toggleCapture);
  connect(evaluate, &QPushButton::clicked, this, &MainWindow::evaluateCalibration);
  connect(stage, &QPushButton::clicked, this, &MainWindow::stageCalibrationResult);
  connect(export_button, &QPushButton::clicked, this, &MainWindow::exportCalibrationReport);
  connect(clear, &QPushButton::clicked, this, [this] {
    if (calibration_step_ == 0) gyro_samples_.clear(); else if (calibration_step_ == 1) accel_samples_.clear();
    else if (calibration_step_ == 2) { mag_samples_.clear(); mag_accel_samples_.clear(); }
    else accel_samples_.clear();
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
  if (!register_table_) return;
  const QSignalBlocker blocker(register_table_);
  register_table_->setRowCount(0);
  const QString filter = register_filter_ ? register_filter_->text().trimmed() : QString();
  for (const auto& definition : definitions_) {
    const QString searchable = QString("%1 %2 %3 %4").arg(definition.id).arg(definition.group, definition.name, definition.description);
    if (!filter.isEmpty() && !searchable.contains(filter, Qt::CaseInsensitive)) continue;
    const int row = register_table_->rowCount(); register_table_->insertRow(row);
    const QString entries[] = {QString::number(definition.id), definition.group, definition.name,
                               device_values_.value(definition.id), staged_values_.value(definition.id), definition.description};
    for (int column = 0; column < 6; ++column) {
      auto* item = new QTableWidgetItem(entries[column]);
      if (column != 4 || !definition.writable) item->setFlags(item->flags() & ~Qt::ItemIsEditable);
      register_table_->setItem(row, column, item);
    }
  }
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
  if (recording_) recorded_telemetry_.push_back(sample);
  if (capturing_) {
    const double t = capture_clock_.elapsed()/1000.0;
    if (calibration_step_ == 0)
      gyro_samples_.push_back({t, {sample.gx, sample.gy, sample.gz}, sample.temperature_c});
    else if (calibration_step_ == 1 || calibration_step_ == 3)
      accel_samples_.push_back({t, {sample.ax, sample.ay, sample.az}, sample.temperature_c});
    else {
      mag_samples_.push_back({t, {sample.mx, sample.my, sample.mz}, sample.temperature_c});
      mag_accel_samples_.push_back({t, {sample.ax, sample.ay, sample.az}, sample.temperature_c});
    }
  }
  if (monitor_paused_) return;
  attitude_->setQuaternion(sample.qx, sample.qy, sample.qz, sample.qw); plot_->append(sample);
  numeric_data_->setText(QString("Acceleration  %1  %2  %3 m/s²\nAngular rate  %4  %5  %6 rad/s\nMagnetic      %7  %8  %9 G\nTemperature   %10 °C\nPressure      %11 kPa")
      .arg(sample.ax,0,'f',4).arg(sample.ay,0,'f',4).arg(sample.az,0,'f',4)
      .arg(sample.gx,0,'f',6).arg(sample.gy,0,'f',6).arg(sample.gz,0,'f',6)
      .arg(sample.mx,0,'f',4).arg(sample.my,0,'f',4).arg(sample.mz,0,'f',4)
      .arg(sample.temperature_c,0,'f',2).arg(sample.pressure_kpa,0,'f',3));
}

void MainWindow::onRegisterRead(int id, const QString& values) {
  device_values_[id] = values; if (staged_values_.value(id) == values) staged_values_.remove(id); refreshRegisterTable();
}

void MainWindow::readSelectedRegister() {
  const int row = register_table_->currentRow(); if (row < 0) return; emit requestRead(register_table_->item(row,0)->text().toInt());
}

void MainWindow::applyStagedRegisters() {
  if (staged_values_.isEmpty()) { setStatus("No staged changes"); return; }
  for (auto iterator = staged_values_.cbegin(); iterator != staged_values_.cend(); ++iterator) {
    QString error; const auto definition = definitionForId(iterator.key());
    if (definition.id < 0) { setStatus(QString("Register %1 is unsupported by this schema").arg(iterator.key()), true); return; }
    if (!validateRegisterValue(definition, iterator.value(), &error)) { setStatus(definition.name + ": " + error, true); return; }
    if (definition.dangerous && QMessageBox::question(this, "Connection-affecting register", definition.name + " can interrupt the connection. Apply it?") != QMessageBox::Yes) return;
  }
  for (auto iterator = staged_values_.cbegin(); iterator != staged_values_.cend(); ++iterator) emit requestWrite(iterator.key(), iterator.value());
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
  if (QMessageBox::question(this, "Save all active settings", "The sensor save command persists its complete active configuration, not only registers shown here. Save to flash?") == QMessageBox::Yes) emit requestSaveSettings();
}

void MainWindow::toggleCapture() {
  capturing_ = !capturing_; if (capturing_) capture_clock_.restart();
  capture_button_->setText(capturing_ ? "Stop capture" : "Start capture");
  calibration_result_->appendPlainText(capturing_ ? "Capture started. Keep each requested pose stationary; stop between motions." : "Capture stopped; no sensor values changed.");
}

void MainWindow::evaluateCalibration() {
  if (capturing_) toggleCapture();
  if (calibration_step_ == 0) { gyro_result_ = estimateGyroBias(gyro_samples_, policy_); calibration_result_->appendPlainText(QString::fromStdString(gyro_result_.quality.summary) + "\nResidual " + vectorText(gyro_result_.residual_bias)); }
  else if (calibration_step_ == 1) { accel_result_ = estimateAccelerometerBias(accel_samples_, policy_); calibration_result_->appendPlainText(QString::fromStdString(accel_result_.quality.summary) + "\nBias " + vectorText(accel_result_.bias) + QString("; measured magnitude %1 ± %2 m/s²").arg(accel_result_.measured_gravity).arg(accel_result_.gravity_stddev)); }
  else if (calibration_step_ == 2) { mag_result_ = estimatePlanarMagBias(mag_samples_, mag_accel_samples_, policy_); calibration_result_->appendPlainText(QString::fromStdString(mag_result_.quality.summary) + "\nObservable in-plane bias " + vectorText(mag_result_.in_plane_bias)); }
  else { gravity_report_ = makeGravityReport(accel_samples_, policy_); calibration_result_->appendPlainText(QString::fromStdString(gravity_report_.quality.summary) + "\nSpecific force " + vectorText(gravity_report_.specific_force_mean) + "\nGravity vector (opposite specific force) " + vectorText(gravity_report_.gravity_vector) + QString("\nMagnitude %1 ± %2 m/s²; tilt from sensor +Z %3 ± %4 deg. This is not sensor-to-robot mounting rotation.").arg(gravity_report_.magnitude).arg(gravity_report_.magnitude_stddev).arg(gravity_report_.tilt_from_sensor_z_deg).arg(gravity_report_.tilt_stddev_deg)); }
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
  report["gyro"] = QJsonObject{{"passed",gyro_result_.quality.passed},{"residual_bias_rad_s",vec(gyro_result_.residual_bias)},{"quality",QString::fromStdString(gyro_result_.quality.summary)}};
  report["accelerometer"] = QJsonObject{{"passed",accel_result_.quality.passed},{"bias_m_s2",vec(accel_result_.bias)},{"measured_magnitude_m_s2",accel_result_.measured_gravity},{"quality",QString::fromStdString(accel_result_.quality.summary)}};
  report["magnetometer"] = QJsonObject{{"passed",mag_result_.quality.passed},{"in_plane_bias_gauss",vec(mag_result_.in_plane_bias)},{"coverage_deg",mag_result_.coverage_deg},{"quality",QString::fromStdString(mag_result_.quality.summary)}};
  report["gravity_report"] = QJsonObject{{"passed",gravity_report_.quality.passed},{"specific_force_mean_m_s2",vec(gravity_report_.specific_force_mean)},{"gravity_vector_m_s2",vec(gravity_report_.gravity_vector)},{"magnitude_m_s2",gravity_report_.magnitude},{"tilt_from_sensor_z_deg",gravity_report_.tilt_from_sensor_z_deg},{"quality",QString::fromStdString(gravity_report_.quality.summary)}};
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
