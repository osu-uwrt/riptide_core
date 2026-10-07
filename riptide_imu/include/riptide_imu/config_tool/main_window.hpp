#pragma once

#include <QElapsedTimer>
#include <QMainWindow>
#include <QMap>
#include <QThread>
#include <deque>

#include "riptide_imu/config_tool/calibration.hpp"
#include "riptide_imu/config_tool/configuration.hpp"
#include "riptide_imu/config_tool/sensor_transport.hpp"

class QLabel;
class QComboBox;
class QLineEdit;
class QListWidget;
class QPlainTextEdit;
class QPushButton;
class QStackedWidget;
class QTableWidget;
class QTimer;

namespace riptide_imu {

class AttitudeWidget : public QWidget {
  Q_OBJECT
 public:
  explicit AttitudeWidget(QWidget* parent = nullptr);
  void setQuaternion(double x, double y, double z, double w);
 protected:
  void paintEvent(QPaintEvent*) override;
 private:
  Eigen::Quaterniond orientation_{Eigen::Quaterniond::Identity()};
};

class PlotWidget : public QWidget {
  Q_OBJECT
 public:
  explicit PlotWidget(QWidget* parent = nullptr);
  void append(const Telemetry& sample);
 protected:
  void paintEvent(QPaintEvent*) override;
 private:
  std::deque<Telemetry> samples_;
};

class MainWindow : public QMainWindow {
  Q_OBJECT
 public:
  MainWindow(QString imu_port, int baud, QString config_file, bool demo,
             QString share_directory, QWidget* parent = nullptr);
  ~MainWindow() override;

 signals:
  void requestConnect(const QString& port, int baud);
  void requestDisconnect();
  void requestDemo(bool enabled);
  void requestRead(int id);
  void requestWrite(int id, const QString& values);
  void requestSaveSettings();
  void requestTare();
  void requestReset();

 private slots:
  void onTelemetry(const Telemetry& sample);
  void onRegisterRead(int id, const QString& values);
  void refreshSerialPorts();
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

 private:
  QWidget* buildMonitorPage();
  QWidget* buildRegistersPage();
  QWidget* buildCalibrationPage();
  QWidget* buildDiagnosticsPage();
  QWidget* buildConnectionBar(const QString& initial_port, int initial_baud);
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
  SensorTransport* transport_{nullptr};
  QStackedWidget* pages_{nullptr};
  QListWidget* navigation_{nullptr};
  QComboBox* port_selector_{nullptr};
  QComboBox* baud_selector_{nullptr};
  QPushButton* connection_button_{nullptr};
  QLabel* status_{nullptr};
  QLabel* numeric_data_{nullptr};
  AttitudeWidget* attitude_{nullptr};
  PlotWidget* plot_{nullptr};
  QTableWidget* register_table_{nullptr};
  QLineEdit* register_filter_{nullptr};
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
  GyroResult gyro_result_;
  AccelerometerResult accel_result_;
  PlanarMagResult mag_result_;
  GravityReport gravity_report_;
  CalibrationPolicy policy_;
};

}  // namespace riptide_imu
