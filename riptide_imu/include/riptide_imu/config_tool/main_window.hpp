#pragma once

#include <QElapsedTimer>
#include <QMainWindow>
#include <QMap>
#include <QPointer>
#include <QThread>
#include <deque>
#include <functional>

#include "riptide_imu/config_tool/calibration.hpp"
#include "riptide_imu/config_tool/calibration_guide.hpp"
#include "riptide_imu/config_tool/calibration_wizard.hpp"
#include "riptide_imu/config_tool/configuration.hpp"
#include "riptide_imu/config_tool/dashboard_widgets.hpp"
#include "riptide_imu/config_tool/output_bandwidth.hpp"
#include "riptide_imu/config_tool/register_editors.hpp"
#include "riptide_imu/config_tool/sensor_transport.hpp"

class QAction;
class QLabel;
class QComboBox;
class QLineEdit;
class QPlainTextEdit;
class QPushButton;
class QSplitter;
class QStackedWidget;
class QTableWidget;
class QTabWidget;
class QTimer;
class QTreeWidget;
class QTreeWidgetItem;

namespace riptide_imu {

class MainWindow : public QMainWindow {
  Q_OBJECT
 public:
  MainWindow(QString imu_port, int baud, QString config_file, bool demo,
             QString share_directory, QWidget* parent = nullptr,
             SensorInterface* injected_sensor = nullptr,
             QString connection_mode = "serial");
  ~MainWindow() override;

  CalibrationWizard* calibrationWizard() const { return wizard_; }
  const CalibrationSession& calibrationSession() const { return calibration_; }
  const QMap<int, QString>& stagedValues() const { return staged_values_; }
  // Line load of the sensor's output configuration, as the sensor has it or with staged values applied.
  BandwidthEstimate bandwidth(bool with_staged) const;

 signals:
  void requestConnect(const QString& port, int baud);
  void requestDisconnect();
  void requestConnectionMode(const QString& mode);
  void requestEndpointDiscovery();
  void requestDemo(bool enabled);
  void requestCalibrationCapture(bool enabled);
  void requestOpenMaintenance();
  void requestCloseMaintenance();
  void requestRead(int id);
  void requestWrite(int id, const QString& values);
  void requestSaveSettings(const QString& expected_json);
  void requestTare();
  void requestReset();

 public slots:
  void openCalibrationWizard();

 private slots:
  void onTelemetry(const Telemetry& sample);
  void onCalibrationSample(const Telemetry& sample);
  void onRegisterRead(int id, const QString& values);
  void refreshSerialPorts();
  // Rediscovers robots or rescans serial devices; a found single robot is connected only if `auto_connect`.
  void refreshConnectionTargets(bool auto_connect = true);
  void toggleConnection();
  void readSelectedRegister();
  void applyStagedRegisters();
  void discardStagedRegisters();
  void importConfiguration();
  void exportConfiguration();
  void savePersistently();
  void resetSensor();
  void exportCalibrationReport();
  void exportTelemetry();
  void loadReplay();
  void selectRegister(QTreeWidgetItem* current, QTreeWidgetItem* previous);
  void stageInspectorValues();
  void resetDashboardLayout();
  void stageCalibration(riptide_imu::CalibrationStep step);
  void stageLatestCalibration();
  void exportProtocolLog();

 private:
  QWidget* buildBoard(const QString& initial_port, int initial_baud);
  QWidget* buildCommandBar();
  QWidget* buildRegisterListPage();
  QWidget* buildInspectorPage();
  QWidget* buildAttitudePage();
  QWidget* buildSignalsPage();
  QWidget* buildCalibrationPage();
  QWidget* buildProtocolPage();
  void buildMenus();
  void applyDefaultLayout();
  void restoreLayout();
  void setTheme(const QString& id);
  void setConnectionMode(const QString& mode);
  void refreshRegisterInspector();
  void refreshStagedIndicators();
  void refreshTelemetryChip();
  void refreshCalibrationSummary();
  void refreshBandwidth();
  void registersChanged();
  void readOutputRegisters();
  void scheduleAutoRead();
  QString rateSource() const;
  QString missingCalibrationRegisters() const;
  QStringList fieldNames(const RegisterDefinition& definition, int count) const;
  void loadDefinitions(const QString& path);
  void refreshRegisterTable();
  void setStatus(const QString& text, bool error = false);
  RegisterDefinition definitionForId(int id) const;
  bool confirm(const QString& title, const QString& text, bool warning = false);
  // Over ROS, driver-critical actions need a maintenance session. Without one this asks to enter it, then runs
  // retry once it is active, and returns true so the caller stops.
  bool needsMaintenance(const QString& action, std::function<void()> retry = {});
  void askForMaintenance(const QString& action, std::function<void()> retry);

  QString share_directory_;
  QVector<RegisterDefinition> definitions_;
  QMap<int, QString> device_values_;
  QMap<int, QString> staged_values_;
  SensorConfiguration configuration_metadata_;

  QThread sensor_thread_;
  SensorInterface* transport_{nullptr};
  bool owns_transport_{false};

  // board and command bar
  SegmentedSwitch* connection_mode_selector_{nullptr};
  QComboBox* port_selector_{nullptr};
  QComboBox* baud_selector_{nullptr};
  QLabel* baud_label_{nullptr};
  QLabel* connection_target_label_{nullptr};
  QLabel* context_label_{nullptr};
  QPushButton* connection_button_{nullptr};
  QPushButton* maintenance_button_{nullptr};
  QPushButton* apply_button_{nullptr};
  StatusChip* telemetry_chip_{nullptr};
  StatusChip* maintenance_chip_{nullptr};
  StatusChip* staged_chip_{nullptr};
  BandwidthMeter* bandwidth_meter_{nullptr};
  QLabel* status_{nullptr};

  // panels
  QSplitter* main_splitter_{nullptr};
  QSplitter* center_splitter_{nullptr};
  QSplitter* right_splitter_{nullptr};
  QTabWidget* lower_tabs_{nullptr};
  AttitudeWidget* attitude_{nullptr};
  TelemetrySummaryWidget* telemetry_summary_{nullptr};
  PlotWidget* plot_{nullptr};
  QTreeWidget* register_tree_{nullptr};
  QTableWidget* register_inspector_{nullptr};
  QStackedWidget* inspector_stack_{nullptr};
  BinaryOutputEditor* binary_editor_{nullptr};
  QLineEdit* register_filter_{nullptr};
  QLineEdit* raw_register_editor_{nullptr};
  QPushButton* stage_raw_button_{nullptr};
  QLabel* register_title_{nullptr};
  QLabel* register_description_{nullptr};
  StatusChip* register_group_chip_{nullptr};
  StatusChip* register_access_chip_{nullptr};
  StatusChip* register_state_chip_{nullptr};
  QPlainTextEdit* traffic_log_{nullptr};
  MetricTable* calibration_summary_{nullptr};
  QPushButton* stage_latest_button_{nullptr};
  QLabel* calibration_note_{nullptr};
  QList<QAction*> theme_actions_;

  QPointer<CalibrationWizard> wizard_;
  CalibrationSession calibration_;
  CalibrationPolicy policy_;

  std::vector<Telemetry> recorded_telemetry_;
  std::deque<Telemetry> replay_telemetry_;
  QTimer* replay_timer_{nullptr};
  bool recording_{false};
  bool monitor_paused_{false};
  bool connected_{false};
  bool maintenance_active_{false};
  std::function<void()> after_maintenance_;
  bool discovery_auto_connect_{true};
  QString connection_mode_{"serial"};
  QString connected_target_;
  int current_register_id_{-1};
  bool updating_register_editor_{false};
  QElapsedTimer last_sample_clock_;
  QTimer* stale_timer_{nullptr};
  QTimer* auto_read_timer_{nullptr};
};

}  // namespace riptide_imu
