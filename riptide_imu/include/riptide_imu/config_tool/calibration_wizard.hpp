#pragma once

#include <QDialog>
#include <QElapsedTimer>
#include <array>
#include <deque>
#include <optional>
#include <vector>

#include "riptide_imu/config_tool/calibration_guide.hpp"
#include "riptide_imu/config_tool/dashboard_widgets.hpp"
#include "riptide_imu/config_tool/robot_mount.hpp"
#include "riptide_imu/config_tool/sensor_transport.hpp"

class QCheckBox;
class QComboBox;
class QFrame;
class QLabel;
class QProgressBar;
class QPushButton;
class QStackedWidget;
class QTimer;

namespace riptide_imu {

// One line of a ruled check table: what was measured, against which limit, and whether it is within it.
struct MetricRow {
  QString label;
  QString value;
  QString limit;
  StatusChip::Level level{StatusChip::Level::Neutral};
  QString verdict;
};

class MetricTable : public QWidget {
  Q_OBJECT
 public:
  explicit MetricTable(QWidget* parent = nullptr);
  void setRows(const std::vector<MetricRow>& rows);
  // Column heads and the share of the width each of the first three columns takes.
  void setColumns(const QStringList& heads, const std::array<double, 3>& weights);
  // Off: columns follow the weights alone and long values are cut short, for narrow panels.
  void setFitContent(bool fit) { fit_content_ = fit; updateGeometry(); }
  const std::vector<MetricRow>& rows() const { return rows_; }
  QSize sizeHint() const override;
  // Wide enough for every label, value and limit without eliding.
  QSize minimumSizeHint() const override;

 protected:
  void paintEvent(QPaintEvent*) override;

 private:
  // Label, value and limit column widths at `width`: each at least its content, spare room to the value.
  std::array<double, 3> columnWidths(double width) const;
  double chipColumn() const;
  std::vector<MetricRow> rows_;
  QStringList heads_{"Check", "Measured", "Limit"};
  std::array<double, 3> weights_{0.4, 0.38, 0.22};
  bool fit_content_{true};
};

// The drawing for a step: the VN-100 in the pose to take, with the live pose as a dashed outline.
class GuideIllustration : public QWidget {
  Q_OBJECT
 public:
  enum class Mode { Prepare, Hold, Pose, Turn };
  explicit GuideIllustration(QWidget* parent = nullptr);
  void setMode(Mode mode);
  void setTarget(const Eigen::Vector3d& gravity_in_body);
  void setLive(const std::optional<Eigen::Vector3d>& gravity_in_body);
  void setCaption(const QString& title, const QString& detail);
  // With a robot frame the sensor is drawn as mounted, with the robot's front marked on the ground.
  void setFrame(const GuideFrame& frame);

 protected:
  void paintEvent(QPaintEvent*) override;

 private:
  GuideFrame frame_;
  Mode mode_{Mode::Prepare};
  Eigen::Vector3d target_{Eigen::Vector3d::UnitZ()};
  std::optional<Eigen::Vector3d> live_;
  QString title_, detail_;
  QTimer* animation_{nullptr};
  QElapsedTimer clock_;
};

// Whether the sensor is at rest, against the thresholds the capture uses.
class StillnessMeter : public QWidget {
  Q_OBJECT
 public:
  explicit StillnessMeter(QWidget* parent = nullptr);
  void setReading(const StillnessMonitor& monitor);
  QSize sizeHint() const override { return {320, 54}; }

 protected:
  void paintEvent(QPaintEvent*) override;

 private:
  double ratio_{0.0};
  bool still_{false};
  bool has_data_{false};
};

// Top-down disc of the sensor's tilt: each accelerometer pose is a ring, the live low point a dot.
class TiltTargetDisc : public QWidget {
  Q_OBJECT
 public:
  explicit TiltTargetDisc(QWidget* parent = nullptr);
  void setTargets(const std::vector<TiltTarget>& targets, double tolerance_deg);
  void setState(const std::vector<bool>& captured, int active, int suggested, double hold_progress);
  void setLive(const std::optional<Eigen::Vector3d>& gravity_in_body);
  void setFrame(const GuideFrame& frame);
  QSize sizeHint() const override { return {300, 300}; }
  int heightForWidth(int width) const override { return width; }
  bool hasHeightForWidth() const override { return true; }

 protected:
  void paintEvent(QPaintEvent*) override;

 private:
  QPointF place(const Eigen::Vector3d& direction, const QPointF& center, double radius) const;
  GuideFrame frame_;
  std::vector<TiltTarget> targets_;
  std::vector<bool> captured_;
  int active_{-1}, suggested_{-1};
  double hold_progress_{0.0};
  double tolerance_deg_{12.0};
  std::optional<Eigen::Vector3d> live_;
};

// Heading turned so far as a ring of 10-degree sectors, with a needle at the current heading.
class HeadingCoverageRing : public QWidget {
  Q_OBJECT
 public:
  explicit HeadingCoverageRing(QWidget* parent = nullptr);
  void setCoverage(const HeadingCoverage& coverage, double required_deg, bool active);
  QSize sizeHint() const override { return {300, 300}; }

 protected:
  void paintEvent(QPaintEvent*) override;

 private:
  std::array<bool, HeadingCoverage::kSectors> sectors_{};
  double heading_deg_{0.0}, coverage_deg_{0.0}, required_deg_{300.0};
  bool active_{false};
};

// The steps down the side, numbered, with each one's state.
class WizardStepper : public QWidget {
  Q_OBJECT
 public:
  struct Entry {
    QString title;
    QString state;
    StatusChip::Level level{StatusChip::Level::Neutral};
    bool enabled{true};
  };
  explicit WizardStepper(QWidget* parent = nullptr);
  void setEntries(const std::vector<Entry>& entries, int current);
  QSize sizeHint() const override { return {230, 400}; }

 signals:
  void stepClicked(int index);

 protected:
  void paintEvent(QPaintEvent*) override;
  void mousePressEvent(QMouseEvent* event) override;

 private:
  std::vector<Entry> entries_;
  int current_{0};
};

// One-line result for a step ("Residual [..] rad/s"), shared with the dashboard's calibration panel.
QString calibrationHeadline(const CalibrationSession& session, CalibrationStep step);

// The guided calibration, in its own window. It only gathers and evaluates samples; staging (copying a result
// into the register workspace) is the main window's job and is never a sensor write.
class CalibrationWizard : public QDialog {
  Q_OBJECT
 public:
  enum Page { Prepare = 0, Gyro, Accelerometer, Magnetometer, Gravity, Review, PageCount };
  // Staged and Applied are a passed step whose result is in the register workspace, or already on the sensor.
  enum class StepState { NotRun, Skipped, Capturing, Passed, Failed, Staged, Applied };

  CalibrationWizard(CalibrationSession* session, const CalibrationPolicy& policy, QWidget* parent = nullptr);

  void addSample(const Telemetry& sample);
  void setConnected(bool connected);
  // Comma-separated register ids still unread; empty when every compensation register is known.
  void setRegistersMissing(const QString& missing);
  void setStageOutcome(CalibrationStep step, bool staged, const QString& message);
  // The register workspace, so results show what they would change and whether they are staged or applied.
  void setRegisterContext(const QMap<int, QString>& device, const QMap<int, QString>& staged);
  int currentPage() const;
  StepState stepState(CalibrationStep step) const;
  const GuideFrame& guideFrame() const { return frame_; }
  const std::optional<RobotMount>& robotMount() const { return mount_; }
  // Uses a robot config for directions (and remembers it), or the sensor's own axes for an empty path.
  bool useRobotConfig(const QString& path);

 public slots:
  void goToPage(int page);
  void startCapture();
  void cancelCapture();

 signals:
  void captureRequested(bool enabled);
  void registersRequested();
  void stageRequested(riptide_imu::CalibrationStep step);
  void exportRequested();
  void sessionChanged();

 protected:
  void showEvent(QShowEvent* event) override;
  void hideEvent(QHideEvent* event) override;

 private:
  enum class Phase { Idle, Settling, Capturing, Done };
  struct StepPage {
    QWidget* page{nullptr};
    GuideIllustration* illustration{nullptr};
    QLabel* instructions{nullptr};
    QLabel* status{nullptr};
    QProgressBar* progress{nullptr};
    QPushButton* action{nullptr};
    MetricTable* live{nullptr};
    MetricTable* result{nullptr};  // checks with limits: whether the capture can be trusted
    QLabel* verdict{nullptr};
    QLabel* advice{nullptr};
    QLabel* checks_title{nullptr};
    QLabel* output_title{nullptr};
    QFrame* output{nullptr};  // the result itself: what it measured and what it would change
    QLabel* output_heading{nullptr};
    QLabel* output_values{nullptr};
    QLabel* output_note{nullptr};
    QPushButton* output_action{nullptr};
  };

  QWidget* buildPreparePage();
  QWidget* buildStepPage(Page page, const QString& title, const QStringList& instructions,
                         QWidget* gauge);
  QWidget* buildReviewPage();
  void refreshGuideFrame();
  void refreshRobotChoices();
  void refreshOutput(Page page);
  QStringList instructionsFor(Page page) const;
  StepPage& stepPage(Page page) { return step_pages_[page]; }
  static CalibrationStep stepFor(Page page) { return static_cast<CalibrationStep>(page - Gyro); }
  bool isCapturePage(int page) const { return page >= Gyro && page <= Gravity; }
  bool stepSelected(int page) const;
  int nextPage(int from, int direction) const;

  void handleHoldSample(const Telemetry& sample, double t, double duration_s);
  void handlePoseSample(const Telemetry& sample, double t);
  void handleTurnSample(const Telemetry& sample, double t);
  void finishCapture();
  void setPhase(Phase phase, const QString& message, const QString& tone = "muted");
  void refreshLive();
  void refreshResult(Page page);
  void refreshPrepare();
  void refreshReview();
  void refreshNavigation();
  std::vector<MetricRow> resultRows(CalibrationStep step) const;
  QString adviceFor(CalibrationStep step) const;
  static Sample toSample(double t, const Eigen::Vector3d& value, double temperature);

  CalibrationSession* session_;
  CalibrationPolicy policy_;
  std::array<StepState, kCalibrationStepCount> states_{};
  QString stage_messages_[kCalibrationStepCount];

  WizardStepper* stepper_{nullptr};
  QStackedWidget* pages_{nullptr};
  QLabel* header_step_{nullptr};
  StatusChip* header_chip_{nullptr};
  QPushButton* back_{nullptr};
  QPushButton* next_{nullptr};
  QPushButton* skip_{nullptr};
  std::array<StepPage, PageCount> step_pages_{};
  std::array<QCheckBox*, kCalibrationStepCount> selected_{};

  // Prepare page
  StatusChip* data_chip_{nullptr};
  StatusChip* registers_chip_{nullptr};
  StatusChip* warmup_chip_{nullptr};
  QLabel* data_detail_{nullptr};
  QLabel* registers_detail_{nullptr};
  QLabel* warmup_detail_{nullptr};
  QPushButton* reread_{nullptr};
  QComboBox* robot_selector_{nullptr};
  QLabel* robot_detail_{nullptr};
  QPushButton* robot_remove_{nullptr};

  // Gauges
  StillnessMeter* gyro_meter_{nullptr};
  StillnessMeter* gravity_meter_{nullptr};
  StillnessMeter* pose_meter_{nullptr};
  TiltTargetDisc* tilt_disc_{nullptr};
  HeadingCoverageRing* coverage_ring_{nullptr};

  // Review page
  MetricTable* review_table_{nullptr};
  QLabel* review_note_{nullptr};
  QPushButton* stage_button_{nullptr};

  // Directions: the sensor's own axes, or a robot's from its config
  GuideFrame frame_;
  std::optional<RobotMount> mount_;
  QMap<int, QString> device_registers_, staged_registers_;

  // Live state
  bool connected_{false};
  QString registers_missing_{"23, 25, 26, 43, 84"};
  Phase phase_{Phase::Idle};
  QElapsedTimer clock_;
  QElapsedTimer data_clock_;
  double now_{0.0};
  int data_samples_{0};
  double data_rate_hz_{0.0};
  StillnessMonitor monitor_;
  std::optional<Telemetry> last_sample_;
  std::deque<std::pair<double, double>> temperatures_;
  // The temperature verdict only flips after the new one has held for a while, so it never flickers.
  bool temperature_steady_{true};
  bool temperature_known_{false};
  double temperature_candidate_since_{-1.0};
  double last_prepare_refresh_{-1.0};
  double capture_start_{0.0};
  double min_temperature_{0.0}, max_temperature_{0.0};
  int restarts_{0};
  // accelerometer poses
  std::vector<TiltTarget> poses_;
  std::vector<bool> captured_;
  int holding_{-1};
  double hold_start_{0.0};
  std::vector<Sample> hold_buffer_;
  // magnetometer turn
  HeadingCoverage coverage_;
};

}  // namespace riptide_imu
