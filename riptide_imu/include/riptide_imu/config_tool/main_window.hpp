#pragma once

#include <QElapsedTimer>
#include <QMainWindow>
#include <QMap>
#include <QThread>
#include <deque>

#include "riptide_imu/config_tool/calibration.hpp"
#include "riptide_imu/config_tool/configuration.hpp"
#include "riptide_imu/config_tool/dashboard_widgets.hpp"
#include "riptide_imu/config_tool/sensor_transport.hpp"

class QAction;
class QLabel;
class QComboBox;
class QDockWidget;
class QLineEdit;
class QPlainTextEdit;
class QPushButton;
class QTableWidget;
class QTimer;
class QTreeWidget;
class QTreeWidgetItem;
class QShowEvent;

namespace riptide_imu {

class MainWindow : public QMainWindow {
  Q_OBJECT
 public:
  MainWindow(QString imu_port, int baud, QString config_file, bool demo,
             QString share_directory, QWidget* parent = nullptr,
             SensorInterface* injected_sensor = nullptr,
             QString connection_mode = "serial");
  ~MainWindow() override;

 protected:
  void showEvent(QShowEvent* event) override;

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

 private slots:
  void onTelemetry(const Telemetry& sample);
  void onCalibrationSample(const Telemetry& sample);
  void onRegisterRead(int id, const QString& values);
  void refreshSerialPorts();
  void refreshConnectionTargets();
  void toggleConnection();
  void readSelectedRegister();
  void applyStagedRegisters();
  void discardStagedRegisters();
  void importConfiguration();
  void exportConfiguration();
  void savePersistently();
  void toggleCapture();
  void evaluateCalibration();
  void stageCalibrationResult();
  void exportCalibrationReport();
  void exportTelemetry();
  void loadReplay();
  void selectRegister(QTreeWidgetItem* current, QTreeWidgetItem* previous);
  void stageInspectorValues();
  void resetDashboardLayout();

 private:
  QWidget* buildMonitorPage();
  QWidget* buildPlotPanel();
  QWidget* buildRegistersPage();
  QWidget* buildCalibrationPage();
  QWidget* buildDiagnosticsPage();
  QWidget* buildConnectionBar(const QString& initial_port, int initial_baud);
  void buildMenusAndToolbar();
  void applyDefaultDashboardLayout();
  void restoreDashboardLayout();
  void refreshRegisterInspector();
  QStringList fieldNames(const RegisterDefinition& definition, int count) const;
  void loadDefinitions(const QString& path);
  void refreshRegisterTable();
  void setStatus(const QString& text, bool error = false);
  RegisterDefinition definitionForId(int id) const;
  static bool parseCompensation(const QString& text, Eigen::Matrix3d* matrix,
                                Eigen::Vector3d* bias);
  static QString formatCompensation(const Eigen::Matrix3d& matrix,
                                    const Eigen::Vector3d& bias);

  QString share_directory_;
  QVector<RegisterDefinition> definitions_;
  QMap<int, QString> device_values_;
  QMap<int, QString> staged_values_;
  SensorConfiguration configuration_metadata_;

  QThread sensor_thread_;
  SensorInterface* transport_{nullptr};
  bool owns_transport_{false};
  QDockWidget* monitor_dock_{nullptr};
  QDockWidget* plot_dock_{nullptr};
  QDockWidget* calibration_dock_{nullptr};
  QDockWidget* diagnostics_dock_{nullptr};
  QComboBox* connection_mode_selector_{nullptr};
  QComboBox* port_selector_{nullptr};
  QComboBox* baud_selector_{nullptr};
  QLabel* baud_label_{nullptr};
  QPushButton* connection_button_{nullptr};
  QPushButton* maintenance_button_{nullptr};
  QLabel* connection_target_label_{nullptr};
  QLabel* status_{nullptr};
  QLabel* numeric_data_{nullptr};
  QLabel* register_description_{nullptr};
  QLabel* register_state_{nullptr};
  AttitudeWidget* attitude_{nullptr};
  TelemetrySummaryWidget* telemetry_summary_{nullptr};
  PlotWidget* plot_{nullptr};
  QTreeWidget* register_tree_{nullptr};
  QTableWidget* register_inspector_{nullptr};
  QLineEdit* register_filter_{nullptr};
  QLineEdit* raw_register_editor_{nullptr};
  QPlainTextEdit* traffic_log_{nullptr};
  QPushButton* capture_button_{nullptr};
  QPlainTextEdit* calibration_result_{nullptr};
  int calibration_step_{0};
  bool capturing_{false};
  QElapsedTimer capture_clock_;
  std::vector<Sample> gyro_samples_;
  std::vector<Sample> accel_samples_;
  std::vector<Sample> mag_samples_;
  std::vector<Sample> mag_accel_samples_;
  std::vector<Telemetry> recorded_telemetry_;
  std::deque<Telemetry> replay_telemetry_;
  QTimer* replay_timer_{nullptr};
  bool recording_{false};
  bool monitor_paused_{false};
  bool connected_{false};
  bool maintenance_active_{false};
  QString connection_mode_{"serial"};
  int current_register_id_{-1};
  bool updating_register_editor_{false};
  QElapsedTimer last_sample_clock_;
  QTimer* stale_timer_{nullptr};
  GyroResult gyro_result_;
  AccelerometerResult accel_result_;
  PlanarMagResult mag_result_;
  GravityReport gravity_report_;
  CalibrationPolicy policy_;
};

}  // namespace riptide_imu
