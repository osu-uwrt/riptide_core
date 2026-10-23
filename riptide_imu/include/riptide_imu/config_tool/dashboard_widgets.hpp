#pragma once

#include <QElapsedTimer>
#include <QStringList>
#include <QWidget>
#include <Eigen/Geometry>
#include <array>
#include <deque>
#include <optional>

#include "riptide_imu/config_tool/drift_monitor.hpp"
#include "riptide_imu/config_tool/output_bandwidth.hpp"
#include "riptide_imu/config_tool/sensor_transport.hpp"

class QLabel;
class QPainter;

namespace riptide_imu {

// A ruled section heading (text over a heavy rule the width of the panel), as in the Nereus heat sheet.
QLabel* sectionTitle(const QString& text);
QLabel* toneLabel(const QString& text, const QString& tone);

// Compact outlined status chip with uppercase text; on the board it takes the board's colors.
class StatusChip : public QWidget {
  Q_OBJECT
 public:
  enum class Level { Neutral, Info, Ok, Warn, Error, Change };
  explicit StatusChip(const QString& text = {}, Level level = Level::Neutral, bool on_board = false,
                      QWidget* parent = nullptr);
  void setState(const QString& text, Level level);
  // Keeps the chip at least as wide as the widest of `texts`, so changing state never shifts the layout.
  void reserveWidthFor(const QStringList& texts);
  QString text() const { return text_; }
  Level level() const { return level_; }
  QSize sizeHint() const override;
  QSize minimumSizeHint() const override { return sizeHint(); }

 protected:
  void paintEvent(QPaintEvent*) override;

 private:
  QString text_;
  Level level_;
  bool on_board_;
  int reserved_width_{0};
};

// The chip's drawing, for tables that place chips inline.
void paintStatusChip(QPainter& painter, const QRectF& rect, const QString& text, StatusChip::Level level,
                     bool on_board = false);
double statusChipWidth(const QString& text);

// A choice between a few options as one joined control whose thumb sits on the chosen side.
class SegmentedSwitch : public QWidget {
  Q_OBJECT
 public:
  explicit SegmentedSwitch(const QStringList& options, bool on_board = false, QWidget* parent = nullptr);
  int currentIndex() const { return index_; }
  void setCurrentIndex(int index);
  QSize sizeHint() const override;
  QSize minimumSizeHint() const override { return sizeHint(); }

 signals:
  void currentIndexChanged(int index);
  // Every click on an option, including the one already chosen.
  void activated(int index);

 protected:
  void paintEvent(QPaintEvent*) override;
  void mousePressEvent(QMouseEvent* event) override;

 private:
  QRectF segment(int index) const;
  QStringList options_;
  int index_{0};
  bool on_board_;
};

// Camera around the scene origin in a Z-up world (east, north, up).
struct SceneCamera {
  double azimuth_deg{-130.0};
  double elevation_deg{24.0};
  double distance{5.4};
};

struct SceneOptions {
  bool grid{true};
  bool compass{true};
  bool horizon{true};
  bool shadow{true};
  bool axes{true};
  bool decal{true};
  bool turn_arrow{false};  // a curved arrow around the vertical: "turn it in place"
  double ground_z{-1.45};
  // A robot's forward direction in the world, drawn as an arrow on the ground labeled with `forward_label`.
  std::optional<Eigen::Vector3d> forward;
  QString forward_label{"FRONT"};
};

// Paints the VN-100 Rugged model (sensor_model.hpp), its body X / Y / Z axes, the ground grid, the level ring and
// a shadow.
// `body_to_world` maps sensor axes (X forward, Y right, Z down) into the Z-up world. An optional ghost (outline
// only) shows a second orientation, such as a target pose.
void paintSensorScene(QPainter& painter, const QRectF& rect, const Eigen::Matrix3d& body_to_world,
                      const SceneCamera& camera, const SceneOptions& options,
                      const std::optional<Eigen::Matrix3d>& ghost = std::nullopt);

// NED to the Z-up east-north-up drawing world (swap X and Y, negate Z); its own inverse.
Eigen::Matrix3d nedToEnu();
// Roll, pitch, yaw in degrees (3-2-1, as the VN-100 reports them) from a body-to-NED rotation.
Eigen::Vector3d eulerDeg(const Eigen::Matrix3d& body_to_ned);

// Interactive 3D attitude view: drag to orbit, scroll to zoom, double-click to reset.
class AttitudeWidget : public QWidget {
  Q_OBJECT
 public:
  // The quaternion's world frame: NED from the sensor directly, ENU from the ROS driver (which publishes
  // sensor-frame orientation relative to ENU).
  enum class WorldFrame { Ned, Enu };
  explicit AttitudeWidget(QWidget* parent = nullptr);
  void setWorldFrame(WorldFrame frame);
  void setQuaternion(double x, double y, double z, double w);
  void setViewPreset(int preset);  // 0 3D, 1 level, 2 top
  void setFollowHeading(bool follow);
  void clearAttitude();
  Eigen::Vector3d eulerDegrees() const { return eulerDeg(body_to_ned_); }
  bool hasAttitude() const { return has_attitude_; }

 signals:
  // The user dragged the camera away from the preset.
  void orbited();

 protected:
  void paintEvent(QPaintEvent*) override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void mouseDoubleClickEvent(QMouseEvent* event) override;
  void wheelEvent(QWheelEvent* event) override;

 private:
  Eigen::Matrix3d body_to_ned_{Eigen::Matrix3d::Identity()};
  WorldFrame frame_{WorldFrame::Ned};
  SceneCamera camera_;
  bool follow_heading_{false};
  bool has_attitude_{false};
  QPoint drag_origin_;
  SceneCamera drag_camera_;
};

// Roll, pitch, yaw in degrees from a telemetry sample's quaternion, in the sensor's NED convention whichever frame
// the source publishes in.
Eigen::Vector3d telemetryEulerDeg(const Telemetry& sample, AttitudeWidget::WorldFrame frame);

// History of one quantity (attitude, angular rate, acceleration or magnetic field). Long windows draw from a
// thinned copy, so a ten-minute drift trace stays cheap. Clicking an axis in the legend hides it (grayed, still
// showing its value); the scale fits what is shown.
class PlotWidget : public QWidget {
  Q_OBJECT
 public:
  enum Quantity { Attitude = 0, AngularRate, Acceleration, Magnetic };
  explicit PlotWidget(QWidget* parent = nullptr);
  void append(const Telemetry& sample);
  void setQuantity(int quantity);
  int quantity() const { return quantity_; }
  void setWorldFrame(AttitudeWidget::WorldFrame frame);
  void setAxisVisible(int axis, bool visible);
  bool axisVisible(int axis) const { return visible_[axis]; }
  void setWindowSeconds(double seconds);
  double windowSeconds() const { return window_s_; }
  static QStringList axisNames(int quantity);
  QRectF legendRect(int axis) const { return legend_[axis]; }
  void clear();

 protected:
  void paintEvent(QPaintEvent*) override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;

 private:
  struct Point {
    double time_s;
    std::array<Eigen::Vector3d, 4> values;  // by Quantity, attitude unwrapped
  };
  Point pointFor(const Telemetry& sample);
  std::deque<Point> recent_;  // every sample, the last ~25 s
  std::deque<Point> history_;  // one per 0.25 s, the last ~31 min
  std::optional<Eigen::Vector3d> last_euler_;
  Eigen::Vector3d unwrapped_{Eigen::Vector3d::Zero()};
  AttitudeWidget::WorldFrame frame_{AttitudeWidget::WorldFrame::Ned};
  std::array<bool, 3> visible_{{true, true, true}};
  std::array<QRectF, 3> legend_;
  int quantity_{Attitude};
  double window_s_{20.0};
};

// Live readings as a ruled table of figures: each vector's X / Y / Z and norm, temperature, pressure, the sensor's
// output rate against how often this view updates, and attitude drift while the sensor is still.
class TelemetrySummaryWidget : public QWidget {
  Q_OBJECT
 public:
  explicit TelemetrySummaryWidget(QWidget* parent = nullptr);
  void updateSample(const Telemetry& sample);
  void markStale(bool stale);
  void setWorldFrame(AttitudeWidget::WorldFrame frame);
  // The rate the sensor's output configuration produces (0 when unknown), and a note on where this view's
  // samples come from.
  void setSensorRate(double rate_hz, const QString& source);
  void restartDrift();
  bool stale() const { return stale_; }
  double rateHz() const { return sample_rate_hz_; }
  const DriftMonitor& drift() const { return drift_; }
  QSize sizeHint() const override { return {360, 340}; }
  QSize minimumSizeHint() const override { return {300, 320}; }

 signals:
  void freshnessChanged(bool live, double rate_hz);

 protected:
  void paintEvent(QPaintEvent*) override;

 private:
  std::optional<Telemetry> sample_;
  bool stale_{true};
  QElapsedTimer rate_clock_;
  int samples_in_window_{0};
  double sample_rate_hz_{0.0};
  double sensor_rate_hz_{0.0};
  QString rate_source_;
  AttitudeWidget::WorldFrame frame_{AttitudeWidget::WorldFrame::Ned};
  DriftMonitor drift_;
};

// The serial line's load from the sensor's output configuration: a bar against the baud rate, with the staged
// configuration's load beside it when that differs.
class BandwidthMeter : public QWidget {
  Q_OBJECT
 public:
  explicit BandwidthMeter(QWidget* parent = nullptr);
  void setEstimates(const BandwidthEstimate& device, const std::optional<BandwidthEstimate>& staged);
  QSize sizeHint() const override { return {250, 30}; }
  QSize minimumSizeHint() const override { return sizeHint(); }

 protected:
  void paintEvent(QPaintEvent*) override;

 private:
  BandwidthEstimate device_;
  std::optional<BandwidthEstimate> staged_;
};

}  // namespace riptide_imu
