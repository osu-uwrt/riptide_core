#include "riptide_imu/config_tool/calibration_wizard.hpp"

#include <QApplication>
#include <QCheckBox>
#include <QClipboard>
#include <QComboBox>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QProgressBar>
#include <QPushButton>
#include <QScrollArea>
#include <QSettings>
#include <QSignalBlocker>
#include <QStackedWidget>
#include <QStyle>
#include <QTimer>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>

#include "riptide_imu/config_tool/theme.hpp"

namespace riptide_imu {
namespace {

constexpr double kGyroHoldS = 30.0;
constexpr double kGravityHoldS = 10.0;
constexpr double kPoseHoldS = 2.5;
constexpr double kPoseToleranceDeg = 12.0;
constexpr double kPoseTiltDeg = 45.0;
constexpr double kMaxTurnRateDegS = 60.0;
// Temperature trend: steady below the first rate, changing above the second, each only after holding 15 s.
constexpr double kSteadyRateCMin = 0.15;
constexpr double kChangingRateCMin = 0.25;
constexpr double kTemperatureHoldS = 15.0;
constexpr double kTemperatureWindowS = 60.0;

using Level = StatusChip::Level;

QString vec3(const Eigen::Vector3d& v, int precision = 3) {
  return QString("[%1, %2, %3]")
      .arg(v.x(), 0, 'g', precision).arg(v.y(), 0, 'g', precision).arg(v.z(), 0, 'g', precision);
}

QString number(double value, int decimals) { return QString::number(value, 'f', decimals); }

Level within(bool ok) { return ok ? Level::Ok : Level::Error; }

void setTone(QLabel* label, const QString& tone) {
  label->setProperty("tone", tone);
  label->style()->unpolish(label);
  label->style()->polish(label);
}

QLabel* pageTitle(const QString& text) {
  auto* label = new QLabel(text);
  label->setObjectName("pageTitle");
  return label;
}

QLabel* paragraph(const QString& text, const QString& tone = {}) {
  auto* label = new QLabel(text);
  label->setWordWrap(true);
  label->setTextFormat(Qt::RichText);
  if (!tone.isEmpty()) label->setProperty("tone", tone);
  return label;
}

// Numbered instructions as rich text, so wrapped lines hang under their text rather than their number.
QString instructionHtml(const QStringList& items) {
  QString html = "<table cellspacing='0' cellpadding='0'>";
  for (int i = 0; i < items.size(); ++i)
    html += QString("<tr><td valign='top' style='padding:0 10px 9px 0'><b>%1</b></td>"
                    "<td valign='top' style='padding-bottom:9px'>%2</td></tr>").arg(i + 1).arg(items[i]);
  return html + "</table>";
}

// A column that scrolls instead of squeezing its labels when the window is short.
QScrollArea* scrollingColumn(QVBoxLayout* column) {
  auto* content = new QWidget;
  content->setLayout(column);
  // wrapped text squeezes instead of scrolling otherwise
  for (auto* label : content->findChildren<QLabel*>())
    label->setSizePolicy(label->sizePolicy().horizontalPolicy(), QSizePolicy::Minimum);
  for (auto* frame : content->findChildren<QFrame*>("resultCard"))
    frame->setSizePolicy(frame->sizePolicy().horizontalPolicy(), QSizePolicy::Minimum);
  auto* scroll = new QScrollArea;
  scroll->setWidget(content);
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  scroll->viewport()->setAutoFillBackground(false);
  content->setAutoFillBackground(false);
  return scroll;
}

QLabel* instructionList(const QStringList& items) {
  auto* label = new QLabel(instructionHtml(items));
  label->setWordWrap(true);
  label->setTextFormat(Qt::RichText);
  return label;
}

// The display frame's attitude (X ahead, Y right, Z down) for a gravity direction in the sensor frame: level is
// display +Z along world down, tilted from there by the shortest rotation. Yaw is arbitrary, kept at north.
Eigen::Matrix3d displayAttitude(const Eigen::Vector3d& gravity_in_body, const GuideFrame& frame) {
  return Eigen::Quaterniond::FromTwoVectors((frame.display_from_sensor * gravity_in_body).normalized(),
                                            Eigen::Vector3d::UnitZ())
      .toRotationMatrix();
}

// The sensor's drawing rotation (body to the Z-up world) for that pose: as mounted in the robot when there is one.
Eigen::Matrix3d poseRotation(const Eigen::Vector3d& gravity_in_body, const GuideFrame& frame = {}) {
  return nedToEnu() * displayAttitude(gravity_in_body, frame) * frame.display_from_sensor;
}

QString signedDeg(double value, int decimals = 2) {
  return QString("%1%2°").arg(value >= 0 ? "+" : "−").arg(std::abs(value), 0, 'f', decimals);
}

}  // namespace

QString calibrationHeadline(const CalibrationSession& session, CalibrationStep step) {
  if (!session.evaluated[static_cast<int>(step)]) {
    const auto count = session.sampleCount(step);
    return count ? QString("%1 samples, not evaluated").arg(count) : QString("No capture");
  }
  switch (step) {
    case CalibrationStep::Gyro:
      return QString("Bias %1 rad/s").arg(vec3(session.gyro_result.residual_bias));
    case CalibrationStep::Accelerometer:
      return QString("Bias %1 m/s² · fitted |g| %2")
          .arg(vec3(session.accelerometer_result.bias))
          .arg(number(session.accelerometer_result.measured_gravity, 3));
    case CalibrationStep::Magnetometer:
      return QString("In-plane bias %1 G · %2° covered")
          .arg(vec3(session.magnetometer_result.in_plane_bias))
          .arg(number(session.magnetometer_result.coverage_deg, 0));
    case CalibrationStep::Gravity:
      return QString("Roll %1, pitch %2 · |g| %3 m/s²")
          .arg(signedDeg(session.gravity_report.roll_deg)).arg(signedDeg(session.gravity_report.pitch_deg))
          .arg(number(session.gravity_report.magnitude, 3));
  }
  return {};
}

MetricTable::MetricTable(QWidget* parent) : QWidget(parent) {
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
}

void MetricTable::setRows(const std::vector<MetricRow>& rows) {
  const bool resized = rows.size() != rows_.size();
  rows_ = rows;
  if (resized) updateGeometry();
  update();
}

void MetricTable::setColumns(const QStringList& heads, const std::array<double, 3>& weights) {
  heads_ = heads;
  weights_ = weights;
  update();
}

QSize MetricTable::sizeHint() const {
  const int height = rows_.empty() ? 0 : 30 + static_cast<int>(rows_.size()) * 30;
  return {std::max(360, minimumSizeHint().width()), height};
}

double MetricTable::chipColumn() const {
  double chip = 0;
  for (const auto& row : rows_)
    if (!row.verdict.isEmpty()) chip = std::max(chip, statusChipWidth(row.verdict) + 8);
  return chip;
}

std::array<double, 3> MetricTable::columnWidths(double width) const {
  const QFontMetrics label(uiFont(14)), value(figuresFont(15)), limit(uiFont(14)), head(strongFont(13));
  std::array<double, 3> natural{head.horizontalAdvance(heads_.value(0)) + 12.0,
                                head.horizontalAdvance(heads_.value(1)) + 12.0,
                                head.horizontalAdvance(heads_.value(2)) + 12.0};
  for (const auto& row : rows_) {
    natural[0] = std::max(natural[0], label.horizontalAdvance(row.label) + 12.0);
    natural[1] = std::max(natural[1], value.horizontalAdvance(row.value) + 14.0);
    natural[2] = std::max(natural[2], limit.horizontalAdvance(row.limit) + 12.0);
  }
  // spare room by weight, never below content
  const double available = width - chipColumn();
  if (!fit_content_) return {available * weights_[0], available * weights_[1], available * weights_[2]};
  const double spare = available - natural[0] - natural[1] - natural[2];
  if (spare <= 0) return natural;
  std::array<double, 3> widths;
  for (int i = 0; i < 3; ++i) widths[i] = std::max(natural[i], available * weights_[i]);
  // bumped columns overshoot; take it back from the others' slack, value last
  double over = widths[0] + widths[1] + widths[2] - available;
  for (int i : {2, 0, 1}) {
    if (over <= 0) break;
    const double cut = std::min(over, widths[i] - natural[i]);
    widths[i] -= cut;
    over -= cut;
  }
  return widths;
}

QSize MetricTable::minimumSizeHint() const {
  if (!fit_content_) return {240, rows_.empty() ? 0 : 30 + static_cast<int>(rows_.size()) * 30};
  const auto widths = columnWidths(0);
  return {static_cast<int>(std::ceil(widths[0] + widths[1] + widths[2] + chipColumn())),
          rows_.empty() ? 0 : 30 + static_cast<int>(rows_.size()) * 30};
}

void MetricTable::paintEvent(QPaintEvent*) {
  if (rows_.empty()) return;
  const auto& t = theme();
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  const double w = width();
  const double chip = chipColumn();
  const auto widths = columnWidths(w);
  const double label_w = widths[0], value_w = widths[1], limit_w = widths[2];
  painter.setFont(strongFont(13));
  painter.setPen(t.text);
  painter.drawText(QRectF(0, 0, label_w, 24), Qt::AlignVCenter, heads_.value(0));
  painter.drawText(QRectF(label_w, 0, value_w - 8, 24), Qt::AlignVCenter | Qt::AlignRight, heads_.value(1));
  painter.drawText(QRectF(label_w + value_w, 0, limit_w - 8, 24), Qt::AlignVCenter | Qt::AlignRight,
                   heads_.value(2));
  painter.setPen(QPen(t.text, 2));
  painter.drawLine(QPointF(0, 27), QPointF(w, 27));
  double y = 28;
  for (const auto& row : rows_) {
    painter.setFont(uiFont(14));
    painter.setPen(t.text);
    painter.drawText(QRectF(0, y, label_w, 30), Qt::AlignVCenter,
                     painter.fontMetrics().elidedText(row.label, Qt::ElideRight, label_w - 6));
    painter.setFont(figuresFont(15));
    painter.drawText(QRectF(label_w, y, value_w - 8, 30), Qt::AlignVCenter | Qt::AlignRight,
                     QFontMetrics(figuresFont(15)).elidedText(row.value, Qt::ElideRight, value_w - 10));
    painter.setPen(t.muted);
    painter.drawText(QRectF(label_w + value_w, y, limit_w - 8, 30), Qt::AlignVCenter | Qt::AlignRight, row.limit);
    if (!row.verdict.isEmpty()) {
      const double cw = std::min(chip - 4, statusChipWidth(row.verdict));
      paintStatusChip(painter, QRectF(w - cw, y + 5, cw, 20), row.verdict, row.level);
    }
    y += 30;
    painter.setPen(QPen(t.border, 1));
    painter.drawLine(QPointF(0, y), QPointF(w, y));
  }
}

GuideIllustration::GuideIllustration(QWidget* parent) : QWidget(parent) {
  setMinimumSize(320, 240);
  animation_ = new QTimer(this);
  animation_->setInterval(33);
  connect(animation_, &QTimer::timeout, this, qOverload<>(&QWidget::update));
  clock_.start();
}

void GuideIllustration::setMode(Mode mode) {
  mode_ = mode;
  if (mode_ == Mode::Turn) animation_->start(); else animation_->stop();
  update();
}

void GuideIllustration::setTarget(const Eigen::Vector3d& gravity_in_body) {
  if (target_.isApprox(gravity_in_body)) return;
  target_ = gravity_in_body;
  update();
}

void GuideIllustration::setLive(const std::optional<Eigen::Vector3d>& gravity_in_body) {
  live_ = gravity_in_body;
  update();
}

void GuideIllustration::setFrame(const GuideFrame& frame) {
  frame_ = frame;
  update();
}

void GuideIllustration::setCaption(const QString& title, const QString& detail) {
  if (title == title_ && detail == detail_) return;
  title_ = title;
  detail_ = detail;
  update();
}

void GuideIllustration::paintEvent(QPaintEvent*) {
  const auto& t = theme();
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.fillRect(rect(), t.window);
  SceneCamera camera;
  SceneOptions options;
  options.compass = false;
  // gravity in the sensor frame when level
  const Eigen::Vector3d level = frame_.display_from_sensor.transpose() * Eigen::Vector3d::UnitZ();
  Eigen::Matrix3d body = poseRotation(level, frame_);
  std::optional<Eigen::Matrix3d> ghost;
  switch (mode_) {
    case Mode::Prepare:
      camera.elevation_deg = 52;
      camera.distance = 5.2;
      options.horizon = false;
      break;
    case Mode::Hold:
      options.horizon = false;
      break;
    case Mode::Pose: {
      body = poseRotation(target_, frame_);
      if (live_) ghost = poseRotation(*live_, frame_);
      // look three-quarters across the tilt, low end to the right
      const Eigen::Vector3d tilt = frame_.display_from_sensor * target_;
      const Eigen::Vector3d low =
          nedToEnu() * displayAttitude(target_, frame_) * Eigen::Vector3d(tilt.x(), tilt.y(), 0);
      if (low.head<2>().norm() > 1e-3) camera.azimuth_deg = std::atan2(low.y(), low.x()) * 180.0 / M_PI - 65.0;
      camera.elevation_deg = 30;
      break;
    }
    case Mode::Turn: {
      const double yaw = std::fmod(clock_.elapsed() / 1000.0 * 36.0, 360.0) * M_PI / 180.0;
      // turning about the world vertical, whatever way the sensor sits
      body = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() * body;
      options.turn_arrow = true;
      options.horizon = false;
      camera.elevation_deg = 34;
      break;
    }
  }
  if (frame_.robot) {
    // the robot's front, from the same attitude the sensor is drawn in
    const Eigen::Matrix3d display_to_world = body * frame_.display_from_sensor.transpose();
    options.forward = display_to_world * Eigen::Vector3d::UnitX();
    options.forward_label = frame_.name.isEmpty() ? QString("FRONT") : frame_.name.toUpper() + " FRONT";
  }
  const QRectF scene = QRectF(rect()).adjusted(0, 0, 0, -54);
  paintSensorScene(painter, scene, body, camera, options, ghost);

  if (mode_ == Mode::Hold) {
    const QString text = "HANDS OFF";
    const double w = QFontMetrics(strongFont(13)).horizontalAdvance(text) + 22;
    paintStatusChip(painter, QRectF(width() - w - 12, 12, w, 26), text, Level::Warn);
  }
  if (mode_ == Mode::Pose && ghost) {
    QColor swatch = t.accent;
    swatch.setAlphaF(0.45);
    painter.setPen(Qt::NoPen);
    painter.setBrush(swatch);
    painter.drawRect(QRectF(12, 13, 18, 12));
    painter.setFont(uiFont(12));
    painter.setPen(t.muted);
    painter.drawText(QRectF(38, 10, width() - 50, 18), Qt::AlignLeft,
                     "where the sensor is now; solid: the pose to take");
  }
  const QRectF caption(0, height() - 54, width(), 54);
  painter.fillRect(caption, t.window);
  painter.setPen(QPen(t.border, 1));
  painter.drawLine(caption.topLeft(), caption.topRight());
  painter.setFont(strongFont(16));
  painter.setPen(t.text);
  painter.drawText(caption.adjusted(14, 6, -14, -26), Qt::AlignLeft | Qt::AlignVCenter, title_);
  painter.setFont(uiFont(13));
  painter.setPen(t.muted);
  painter.drawText(caption.adjusted(14, 28, -14, -6), Qt::AlignLeft | Qt::AlignVCenter,
                   painter.fontMetrics().elidedText(detail_, Qt::ElideRight, width() - 28));
}

StillnessMeter::StillnessMeter(QWidget* parent) : QWidget(parent) {
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  setMinimumHeight(54);
}

void StillnessMeter::setReading(const StillnessMonitor& monitor) {
  has_data_ = monitor.hasData();
  still_ = monitor.still();
  ratio_ = std::max(monitor.gyroSpread() / std::max(monitor.gyroLimit(), 1e-12),
                    monitor.accelSpread() / std::max(monitor.accelLimit(), 1e-12));
  update();
}

void StillnessMeter::paintEvent(QPaintEvent*) {
  const auto& t = theme();
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setFont(strongFont(11));
  painter.setPen(t.muted);
  painter.drawText(QRectF(0, 0, width(), 18), Qt::AlignLeft | Qt::AlignVCenter, "MOTION");
  const QString state = !has_data_ ? "No data" : still_ ? "Still" : "Moving";
  const double cw = statusChipWidth(state);
  paintStatusChip(painter, QRectF(width() - cw, 0, cw, 20), state,
                  !has_data_ ? Level::Neutral : still_ ? Level::Ok : Level::Warn);
  const QRectF track(0, 28, width(), 12);
  painter.setPen(QPen(t.border, 1));
  painter.setBrush(t.frame);
  painter.drawRoundedRect(track, 2, 2);
  if (has_data_) {
    const double fraction = std::clamp(ratio_ / 2.0, 0.02, 1.0);
    painter.setPen(Qt::NoPen);
    painter.setBrush(ratio_ <= 1.0 ? t.ok : t.warn);
    painter.drawRoundedRect(QRectF(track.left(), track.top(), track.width() * fraction, track.height()), 2, 2);
  }
  const double limit_x = track.left() + track.width() * 0.5;
  painter.setPen(QPen(t.text, 2));
  painter.drawLine(QPointF(limit_x, track.top() - 3), QPointF(limit_x, track.bottom() + 3));
  painter.setFont(uiFont(11));
  painter.setPen(t.muted);
  painter.drawText(QRectF(limit_x + 4, track.bottom() + 1, 120, 14), Qt::AlignLeft, "capture limit");
}

TiltTargetDisc::TiltTargetDisc(QWidget* parent) : QWidget(parent) {
  QSizePolicy policy(QSizePolicy::Preferred, QSizePolicy::Preferred);
  policy.setHeightForWidth(true);
  setSizePolicy(policy);
  setMinimumSize(200, 200);
}

void TiltTargetDisc::setTargets(const std::vector<TiltTarget>& targets, double tolerance_deg) {
  targets_ = targets;
  tolerance_deg_ = tolerance_deg;
  captured_.assign(targets.size(), false);
  update();
}

void TiltTargetDisc::setState(const std::vector<bool>& captured, int active, int suggested, double hold_progress) {
  captured_ = captured;
  active_ = active;
  suggested_ = suggested;
  hold_progress_ = hold_progress;
  update();
}

void TiltTargetDisc::setLive(const std::optional<Eigen::Vector3d>& gravity_in_body) {
  live_ = gravity_in_body;
  update();
}

void TiltTargetDisc::setFrame(const GuideFrame& frame) {
  frame_ = frame;
  update();
}

QPointF TiltTargetDisc::place(const Eigen::Vector3d& direction, const QPointF& center, double radius) const {
  const Eigen::Vector3d d = (frame_.display_from_sensor * direction).normalized();
  const double tilt = std::acos(std::clamp(d.z(), -1.0, 1.0)) * 180.0 / M_PI;
  const double azimuth = std::atan2(d.y(), d.x());
  const double r = std::min(tilt / 60.0, 1.08) * radius;
  // Top-down on the sensor: +X up the page, +Y to the right.
  return center + QPointF(r * std::sin(azimuth), -r * std::cos(azimuth));
}

void TiltTargetDisc::paintEvent(QPaintEvent*) {
  const auto& t = theme();
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  const double side = std::min(width(), height());
  const QPointF center(width() / 2.0, height() / 2.0);
  // room at the rim for the labels: axis names, or the longer robot words
  const double radius = side / 2.0 - (frame_.robot ? 46 : 26);

  painter.setPen(QPen(t.border, 1));
  painter.setBrush(t.frame);
  painter.drawEllipse(center, radius, radius);
  painter.setBrush(Qt::NoBrush);
  for (double tilt : {15.0, 30.0, 45.0}) {
    QColor ring = t.border;
    ring.setAlphaF(tilt == 45.0 ? 1.0 : 0.6);
    painter.setPen(QPen(ring, 1, tilt == 45.0 ? Qt::SolidLine : Qt::DotLine));
    painter.drawEllipse(center, radius * tilt / 60.0, radius * tilt / 60.0);
  }
  painter.setPen(QPen(t.border, 1));
  painter.drawLine(center - QPointF(radius, 0), center + QPointF(radius, 0));
  painter.drawLine(center - QPointF(0, radius), center + QPointF(0, radius));
  painter.setFont(uiFont(11));
  painter.setPen(t.muted);
  const double label_angle = -M_PI / 8;  // between the +X and the corner ring, clear of both
  const QPointF label_at = center + QPointF(std::sin(label_angle), -std::cos(label_angle)) * radius * 0.75;
  painter.drawText(QRectF(label_at.x() - 16, label_at.y() - 8, 32, 16), Qt::AlignCenter, "45°");

  // rim labels name the low end
  painter.setFont(strongFont(13));
  const QPointF rim[4] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
  for (int i = 0; i < 4; ++i) {
    const QPointF at = center + rim[i] * (radius + 14);
    const double w = QFontMetrics(strongFont(13)).horizontalAdvance(frame_.ends[i]) + 8;
    painter.setPen(frame_.robot ? t.text : i % 2 == 0 ? axisColor(0) : axisColor(1));
    painter.drawText(QRectF(at.x() - w / 2, at.y() - 10, w, 20), Qt::AlignCenter, frame_.ends[i]);
  }

  painter.save();
  painter.translate(center);
  if (frame_.robot) {
    // the robot from above, nose up the page
    painter.setPen(QPen(t.muted, 1.4));
    painter.setBrush(Qt::NoBrush);
    QPolygonF hull;
    hull << QPointF(0, -18) << QPointF(10, -6) << QPointF(10, 16) << QPointF(-10, 16) << QPointF(-10, -6);
    painter.drawPolygon(hull);
  } else {
    // sensor from above
    painter.setPen(QPen(t.muted, 1.2));
    painter.setBrush(Qt::NoBrush);
    painter.drawRect(QRectF(-14, -15, 28, 30));
    painter.setPen(QPen(axisColor(0), 2));
    painter.drawLine(QPointF(0, 0), QPointF(0, -11));
    painter.setPen(QPen(axisColor(1), 2));
    painter.drawLine(QPointF(0, 0), QPointF(10, 0));
  }
  painter.restore();

  const double target_radius = radius * tolerance_deg_ / 60.0;
  for (std::size_t i = 0; i < targets_.size(); ++i) {
    const QPointF at = place(targets_[i].direction(), center, radius);
    const bool done = i < captured_.size() && captured_[i];
    const bool active = static_cast<int>(i) == active_;
    const bool suggested = static_cast<int>(i) == suggested_;
    if (done) {
      painter.setPen(Qt::NoPen);
      QColor fill = t.ok;
      fill.setAlphaF(0.85);
      painter.setBrush(fill);
      painter.drawEllipse(at, target_radius, target_radius);
      painter.setPen(QPen(Qt::white, 2.4, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
      QPolygonF check;
      check << at + QPointF(-6, 0) << at + QPointF(-2, 4) << at + QPointF(6, -5);
      painter.drawPolyline(check);
      continue;
    }
    painter.setBrush(Qt::NoBrush);
    if (active) {
      painter.setPen(QPen(t.accent, 3));
      painter.drawEllipse(at, target_radius, target_radius);
      painter.setPen(QPen(t.ok, 4, Qt::SolidLine, Qt::FlatCap));
      const QRectF arc(at.x() - target_radius - 5, at.y() - target_radius - 5, 2 * target_radius + 10,
                       2 * target_radius + 10);
      painter.drawArc(arc, 90 * 16, -static_cast<int>(hold_progress_ * 360 * 16));
    } else {
      painter.setPen(QPen(suggested ? t.accent : t.muted, suggested ? 2.2 : 1.4, Qt::DashLine));
      painter.drawEllipse(at, target_radius, target_radius);
    }
  }

  if (live_) {
    const QPointF dot = place(*live_, center, radius);
    QColor halo = t.text;
    halo.setAlphaF(0.18);
    painter.setPen(Qt::NoPen);
    painter.setBrush(halo);
    painter.drawEllipse(dot, 12, 12);
    painter.setBrush(t.text);
    painter.drawEllipse(dot, 6, 6);
  }
}

HeadingCoverageRing::HeadingCoverageRing(QWidget* parent) : QWidget(parent) { setMinimumSize(240, 240); }

void HeadingCoverageRing::setCoverage(const HeadingCoverage& coverage, double required_deg, bool active) {
  sectors_ = coverage.sectors();
  heading_deg_ = coverage.headingDeg();
  coverage_deg_ = active ? coverage.coverageDeg() : 0.0;
  required_deg_ = required_deg;
  active_ = active;
  if (!active) sectors_.fill(false);
  update();
}

void HeadingCoverageRing::paintEvent(QPaintEvent*) {
  const auto& t = theme();
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  const double side = std::min(width(), height());
  const QPointF center(width() / 2.0, height() / 2.0);
  const double outer = side / 2.0 - 10, inner = outer * 0.74;
  const QRectF outer_rect(center.x() - outer, center.y() - outer, 2 * outer, 2 * outer);
  const QRectF inner_rect(center.x() - inner, center.y() - inner, 2 * inner, 2 * inner);
  const double span = 360.0 / HeadingCoverage::kSectors;
  for (int i = 0; i < HeadingCoverage::kSectors; ++i) {
    // heading 0 at top going clockwise; Qt angles go counterclockwise from 3 o'clock
    const double start = 90.0 - i * span - 0.8;
    const double sweep = -(span - 1.6);
    QPainterPath path;
    path.arcMoveTo(outer_rect, start);
    path.arcTo(outer_rect, start, sweep);
    path.arcTo(inner_rect, start + sweep, -sweep);
    path.closeSubpath();
    painter.setPen(Qt::NoPen);
    painter.setBrush(sectors_[i] ? t.ok : t.frame);
    painter.drawPath(path);
  }
  painter.setPen(QPen(t.border, 1));
  painter.setBrush(Qt::NoBrush);
  painter.drawEllipse(center, outer + 1, outer + 1);
  if (active_) {
    const double angle = heading_deg_ * M_PI / 180.0;
    const QPointF tip = center + QPointF(std::sin(angle), -std::cos(angle)) * (outer + 6);
    const QPointF base = center + QPointF(std::sin(angle), -std::cos(angle)) * (inner - 8);
    painter.setPen(QPen(t.text, 3, Qt::SolidLine, Qt::RoundCap));
    painter.drawLine(base, tip);
  }
  painter.setPen(active_ && coverage_deg_ >= required_deg_ ? t.ok : t.text);
  painter.setFont(figuresFont(static_cast<int>(std::max(24.0, inner * 0.42))));
  painter.drawText(QRectF(center.x() - inner, center.y() - inner * 0.55, 2 * inner, inner * 0.8), Qt::AlignCenter,
                   active_ ? QString("%1°").arg(coverage_deg_, 0, 'f', 0) : QString("--"));
  painter.setFont(uiFont(13));
  painter.setPen(t.muted);
  painter.drawText(QRectF(center.x() - inner, center.y() + inner * 0.2, 2 * inner, 22), Qt::AlignCenter,
                   QString("of %1° needed").arg(required_deg_, 0, 'f', 0));
}

WizardStepper::WizardStepper(QWidget* parent) : QWidget(parent) {
  setObjectName("wizardStepper");
  setFixedWidth(230);
  setCursor(Qt::PointingHandCursor);
}

void WizardStepper::setEntries(const std::vector<Entry>& entries, int current) {
  entries_ = entries;
  current_ = current;
  update();
}

void WizardStepper::paintEvent(QPaintEvent*) {
  const auto& t = theme();
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.fillRect(rect(), t.window);
  painter.setPen(QPen(t.border, 1));
  painter.drawLine(QPointF(width() - 0.5, 0), QPointF(width() - 0.5, height()));
  const double row = 62;
  const double top = 16;
  for (std::size_t i = 0; i < entries_.size(); ++i) {
    const auto& entry = entries_[i];
    const QRectF box(0, top + i * row, width() - 1, row);
    const QPointF circle(30, box.top() + 24);
    if (i + 1 < entries_.size()) {
      painter.setPen(QPen(t.border, 2));
      painter.drawLine(circle + QPointF(0, 14), QPointF(circle.x(), box.bottom() + 10));
    }
    if (static_cast<int>(i) == current_) {
      painter.fillRect(box.adjusted(0, 0, 0, -6), t.header);
      painter.fillRect(QRectF(0, box.top(), 4, box.height() - 6), t.active);
    }
    QColor ring = t.border, fill = t.window, label = t.muted;
    if (entry.level == Level::Ok) ring = fill = t.ok, label = Qt::white;
    if (entry.level == Level::Error) ring = fill = t.error, label = Qt::white;
    if (entry.level == Level::Change) ring = fill = t.change, label = t.change_text;
    if (static_cast<int>(i) == current_ && entry.level == Level::Neutral)
      ring = fill = t.active, label = t.active_text;
    painter.setPen(QPen(ring, 2));
    painter.setBrush(fill);
    painter.drawEllipse(circle, 13, 13);
    painter.setPen(label);
    painter.setFont(figuresFont(15));
    QString mark = QString::number(i + 1);
    if (entry.level == Level::Ok || entry.level == Level::Change) mark = "✓";
    if (entry.level == Level::Error) mark = "!";
    painter.drawText(QRectF(circle.x() - 13, circle.y() - 13, 26, 26), Qt::AlignCenter, mark);
    painter.setFont(strongFont(15));
    painter.setPen(entry.enabled ? t.text : t.muted);
    painter.drawText(QRectF(54, box.top() + 8, width() - 60, 22), Qt::AlignVCenter, entry.title);
    painter.setFont(uiFont(13));
    painter.setPen(entry.level == Level::Error ? t.error : entry.level == Level::Ok ? t.ok : t.muted);
    painter.drawText(QRectF(54, box.top() + 29, width() - 60, 18), Qt::AlignVCenter, entry.state);
  }
}

void WizardStepper::mousePressEvent(QMouseEvent* event) {
  const int index = static_cast<int>((event->pos().y() - 16) / 62);
  if (index >= 0 && index < static_cast<int>(entries_.size())) emit stepClicked(index);
}

CalibrationWizard::CalibrationWizard(CalibrationSession* session, const CalibrationPolicy& policy, QWidget* parent)
    : QDialog(parent), session_(session), policy_(policy),
      monitor_(1.0, policy.gyro_max_rms_rad_s, 0.5 * policy.accel_max_rms_m_s2) {
  setObjectName("calibrationWizard");
  setWindowTitle("VN-100 Calibration");
  setWindowFlag(Qt::WindowContextHelpButtonHint, false);
  setWindowFlag(Qt::WindowMaximizeButtonHint, true);
  setModal(false);
  resize(1340, 880);
  states_.fill(StepState::NotRun);
  poses_ = accelerometerPoses(kPoseTiltDeg);
  captured_.assign(poses_.size(), false);
  clock_.start();
  data_clock_.start();

  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(0, 0, 0, 0);
  root->setSpacing(0);

  auto* board = new QWidget;
  board->setObjectName("board");
  board->setAttribute(Qt::WA_StyledBackground);
  auto* board_layout = new QHBoxLayout(board);
  board_layout->setContentsMargins(16, 9, 16, 9);
  board_layout->setSpacing(12);
  auto* wordmark = new QLabel("CALIBRATION");
  wordmark->setObjectName("wordmark");
  board_layout->addWidget(wordmark);
  auto* slash = new QLabel("/  VN-100 GUIDED PROCEDURE");
  slash->setProperty("tone", "muted");
  board_layout->addWidget(slash);
  board_layout->addStretch();
  header_step_ = new QLabel;
  board_layout->addWidget(header_step_);
  header_chip_ = new StatusChip("No data", Level::Warn, true);
  board_layout->addWidget(header_chip_);
  root->addWidget(board);

  auto* body = new QHBoxLayout;
  body->setSpacing(0);
  stepper_ = new WizardStepper;
  body->addWidget(stepper_);
  pages_ = new QStackedWidget;
  pages_->setObjectName("wizardPages");
  body->addWidget(pages_, 1);
  root->addLayout(body, 1);

  pages_->addWidget(buildPreparePage());

  gyro_meter_ = new StillnessMeter;
  pages_->addWidget(buildStepPage(Gyro, "Gyro bias", {}, gyro_meter_));

  auto* pose_gauge = new QWidget;
  auto* pose_layout = new QVBoxLayout(pose_gauge);
  pose_layout->setContentsMargins(0, 0, 0, 0);
  tilt_disc_ = new TiltTargetDisc;
  tilt_disc_->setObjectName("tiltTargetDisc");
  tilt_disc_->setTargets(poses_, kPoseToleranceDeg);
  pose_meter_ = new StillnessMeter;
  pose_layout->addWidget(tilt_disc_, 1);
  pose_layout->addWidget(pose_meter_);
  pages_->addWidget(buildStepPage(Accelerometer, "Accelerometer", {}, pose_gauge));

  coverage_ring_ = new HeadingCoverageRing;
  coverage_ring_->setObjectName("headingCoverageRing");
  pages_->addWidget(buildStepPage(Magnetometer, "Magnetometer", {}, coverage_ring_));

  gravity_meter_ = new StillnessMeter;
  pages_->addWidget(buildStepPage(Gravity, "Gravity report", {}, gravity_meter_));

  pages_->addWidget(buildReviewPage());

  auto* footer = new QWidget;
  footer->setObjectName("wizardFooter");
  footer->setAttribute(Qt::WA_StyledBackground);
  auto* footer_layout = new QHBoxLayout(footer);
  footer_layout->setContentsMargins(16, 10, 16, 10);
  skip_ = new QPushButton("Skip this step");
  skip_->setProperty("role", "quiet");
  footer_layout->addWidget(skip_);
  footer_layout->addStretch();
  back_ = new QPushButton("Back");
  next_ = new QPushButton("Next");
  next_->setObjectName("wizardNext");
  next_->setProperty("role", "primary");
  next_->setDefault(true);
  footer_layout->addWidget(back_);
  footer_layout->addWidget(next_);
  root->addWidget(footer);

  connect(stepper_, &WizardStepper::stepClicked, this, [this](int page) {
    if (page == Prepare || page == Review || stepSelected(page)) goToPage(page);
  });
  connect(back_, &QPushButton::clicked, this, [this] { goToPage(nextPage(currentPage(), -1)); });
  connect(next_, &QPushButton::clicked, this, [this] {
    if (currentPage() == Review) {
      close();
      return;
    }
    goToPage(nextPage(currentPage(), 1));
  });
  connect(skip_, &QPushButton::clicked, this, [this] {
    const int page = currentPage();
    if (!isCapturePage(page)) return;
    cancelCapture();
    const int step = static_cast<int>(stepFor(static_cast<Page>(page)));
    if (states_[step] == StepState::NotRun) states_[step] = StepState::Skipped;
    goToPage(nextPage(page, 1));
  });

  auto* refresh = new QTimer(this);
  refresh->setInterval(500);
  connect(refresh, &QTimer::timeout, this, [this] {
    if (data_clock_.elapsed() > 2000) data_rate_hz_ = 0.0;
    refreshPrepare();
    refreshNavigation();
  });
  refresh->start();
  refreshRobotChoices();
  useRobotConfig(QSettings().value("calibration/robot").toString());
  goToPage(Prepare);
}

QStringList CalibrationWizard::instructionsFor(Page page) const {
  const bool robot = frame_.robot;
  switch (page) {
    case Gyro:
      return {"Set the sensor, or the robot with the sensor in it, on a solid table or the floor. Any way up works.",
              "Step back and keep hands off the robot and the table. Nobody walk past; pumps and fans off.",
              "Press <b>Start capture</b>. The hold starts by itself once the sensor is still and runs for "
              "30 seconds; if anything moves, it starts over."};
    case Accelerometer:
      if (robot)
        return {"Press <b>Start capture</b>, then set the robot level. The black dot on the disc shows which part "
                "of the robot is low.",
                "Tip the robot about 45° so the dot moves into a ring, then let go and hold it still. Each ring "
                "fills in after 2.5 seconds of stillness.",
                "Fill all nine rings in any order: level, front, back, each side and each corner down. Prop the "
                "robot on a block if it will not stay tipped on its own."};
      return {"Press <b>Start capture</b>, then lay the sensor flat. The black dot on the disc shows which end of "
              "the sensor is low.",
              "Tip the sensor about 45° so the dot moves into a ring, then let go and hold it still. Each ring "
              "fills in after 2.5 seconds of stillness.",
              "Fill all nine rings in any order: flat, each end, and each corner. Prop the robot on a block if "
              "it will not stay tipped on its own."};
    case Magnetometer:
      return {"Move magnets, steel tools, phones and power cables at least a meter from the sensor.",
              robot ? "Set the robot level, for example on a cart or a turntable, and press <b>Start capture</b>."
                    : "Set the sensor level, for example the robot on a cart or a turntable, and press "
                      "<b>Start capture</b>.",
              "Turn it slowly through a full circle about the vertical, keeping it level. Take about 30 seconds per "
              "turn; the ring fills in as you go."};
    case Gravity:
      return {robot ? "Set the robot down level, the way it normally sits, with the sensor mounted. Check level "
                      "with a spirit level on a surface you trust."
                    : "Set the robot down the way it normally sits, with the sensor mounted.",
              "Press <b>Start capture</b> and keep hands off for 10 seconds.",
              robot ? "The report gives the sensor's roll and pitch against gravity, and the IMU rotation that would "
                      "make this robot read level. It changes no settings and cannot see yaw."
                    : "The report gives the sensor's roll and pitch against gravity. It changes no settings, and it "
                      "cannot see yaw or tell a tilted floor from a tilted mount. Pick a robot config on the "
                      "Prepare page to check the mount against it."};
    default: break;
  }
  return {};
}

void CalibrationWizard::refreshRobotChoices() {
  if (!robot_selector_) return;
  const QSignalBlocker blocker(robot_selector_);
  robot_selector_->clear();
  robot_selector_->addItem("Sensor's own axes (no robot)", QString());
  for (const QString& path : QSettings().value("calibration/robots").toStringList())
    robot_selector_->addItem(QString("%1  —  %2").arg(QFileInfo(path).completeBaseName(), path), path);
  robot_selector_->addItem("Add a robot config…", QString("add"));
  const int current = robot_selector_->findData(mount_ ? mount_->path : QString());
  robot_selector_->setCurrentIndex(std::max(0, current));
  robot_remove_->setEnabled(mount_.has_value());
}

bool CalibrationWizard::useRobotConfig(const QString& path) {
  QSettings settings;
  if (path.isEmpty()) {
    mount_.reset();
    settings.setValue("calibration/robot", QString());
  } else {
    QString error;
    auto mount = loadRobotMount(path, &error);
    if (!mount) {
      if (robot_detail_) {
        robot_detail_->setText(error);
        setTone(robot_detail_, "error");
      }
      refreshRobotChoices();
      return false;
    }
    mount_ = mount;
    QStringList saved = settings.value("calibration/robots").toStringList();
    saved.removeAll(mount_->path);
    saved.prepend(mount_->path);
    settings.setValue("calibration/robots", saved);
    settings.setValue("calibration/robot", mount_->path);
  }
  refreshGuideFrame();
  refreshRobotChoices();
  return true;
}

void CalibrationWizard::refreshGuideFrame() {
  cancelCapture();
  frame_ = mount_ ? robotGuideFrame(mount_->name, mount_->sensor_to_base) : sensorGuideFrame();
  poses_ = accelerometerPoses(kPoseTiltDeg, frame_);
  captured_.assign(poses_.size(), false);
  if (tilt_disc_) {
    tilt_disc_->setTargets(poses_, kPoseToleranceDeg);
    tilt_disc_->setFrame(frame_);
  }
  for (int page = Prepare; page <= Gravity; ++page) {
    auto& p = stepPage(static_cast<Page>(page));
    if (p.illustration) p.illustration->setFrame(frame_);
    if (p.instructions) p.instructions->setText(instructionHtml(instructionsFor(static_cast<Page>(page))));
  }
  auto& prepare = stepPage(Prepare);
  if (prepare.illustration)
    prepare.illustration->setCaption(
        frame_.robot ? QString("Directions are %1's").arg(mount_->name)
                     : QString("Find the arrows printed on the case"),
        frame_.robot ? QString("Front, back, left and right of the robot; the sensor is drawn as mounted.")
                     : QString("Every direction in this wizard means the sensor's own X and Y, not the robot's."));
  if (robot_detail_) {
    if (mount_) {
      QStringList axes;
      for (int axis = 0; axis < 3; ++axis)
        axes << QString("+%1 %2").arg(QChar("XYZ"[axis])).arg(baseDirectionName(mount_->sensor_to_base.col(axis)));
      robot_detail_->setText(QString("Sensor %1. IMU %2 m from base_link (x, y, z). From %3, relative to %4.")
                                 .arg(axes.join(", "))
                                 .arg(QString("%1, %2, %3").arg(mount_->position.x(), 0, 'f', 3)
                                          .arg(mount_->position.y(), 0, 'f', 3).arg(mount_->position.z(), 0, 'f', 3))
                                 .arg(QFileInfo(mount_->path).fileName(), mount_->parent));
    } else {
      robot_detail_->setText("Poses and directions use the sensor's printed X and Y. Pick a robot config to give "
                             "them as the robot's front, back and sides, and to check the mount.");
    }
    setTone(robot_detail_, "muted");
  }
  if (isCapturePage(currentPage())) refreshResult(static_cast<Page>(currentPage()));
  refreshLive();
}

QWidget* CalibrationWizard::buildPreparePage() {
  auto* page = new QWidget;
  page->setObjectName("wizardPage");
  auto* layout = new QHBoxLayout(page);
  layout->setContentsMargins(24, 20, 24, 20);
  layout->setSpacing(28);

  auto& p = stepPage(Prepare);
  p.page = page;
  p.illustration = new GuideIllustration;
  p.illustration->setMode(GuideIllustration::Mode::Prepare);
  p.illustration->setCaption("Find the arrows printed on the case",
                             "Every direction in this wizard means the sensor's own X and Y, not the robot's.");
  layout->addWidget(p.illustration, 4);

  auto* column = new QVBoxLayout;
  column->setSpacing(10);
  column->addWidget(pageTitle("Before you start"));
  column->addWidget(paragraph(
      "Calibration measures the sensor's leftover biases while you hold it in a few positions. Nothing is "
      "written to the sensor until you stage, apply and save the results yourself.", "muted"));
  column->addSpacing(6);
  column->addWidget(sectionTitle("Checks"));

  auto* checks = new QGridLayout;
  checks->setHorizontalSpacing(12);
  checks->setVerticalSpacing(10);
  checks->setColumnStretch(1, 1);
  auto addCheck = [&](int row, StatusChip** chip, QLabel** detail, const QString& title) {
    *chip = new StatusChip("Checking", Level::Neutral);
    auto* text = new QVBoxLayout;
    text->setSpacing(1);
    auto* heading = new QLabel(title);
    heading->setFont(strongFont(14));
    text->addWidget(heading);
    *detail = toneLabel(QString(), "muted");
    text->addWidget(*detail);
    checks->addWidget(*chip, row, 0, Qt::AlignTop | Qt::AlignLeft);
    checks->addLayout(text, row, 1);
  };
  addCheck(0, &data_chip_, &data_detail_, "Calibration data");
  addCheck(1, &registers_chip_, &registers_detail_, "Compensation registers");
  reread_ = new QPushButton("Read again");
  checks->addWidget(reread_, 1, 2, Qt::AlignTop);
  connect(reread_, &QPushButton::clicked, this, &CalibrationWizard::registersRequested);
  addCheck(2, &warmup_chip_, &warmup_detail_, "Steady temperature");
  StatusChip* area_chip = nullptr;
  QLabel* area_detail = nullptr;
  addCheck(3, &area_chip, &area_detail, "Clear the area");
  area_chip->setState("You check", Level::Neutral);
  area_detail->setText("Magnets, steel tools, phones and power cables at least a meter away.");
  // fixed sizes so live updates don't shift anything
  const QStringList chip_texts{"Checking", "999 Hz", "Live", "No data", "Offline", "Read", "Missing", "Watching",
                               "Unknown", "Steady", "Changing", "You check"};
  for (auto* chip : {data_chip_, registers_chip_, warmup_chip_, area_chip}) chip->reserveWidthFor(chip_texts);
  for (auto* detail : {data_detail_, registers_detail_, warmup_detail_, area_detail})
    detail->setMinimumHeight(2 * detail->fontMetrics().lineSpacing() + 2);
  column->addLayout(checks);

  column->addSpacing(8);
  column->addWidget(sectionTitle("Directions"));
  auto* robot_row = new QHBoxLayout;
  robot_selector_ = new QComboBox;
  robot_selector_->setObjectName("robotSelector");
  robot_selector_->setToolTip("A robot description config (such as talos.yaml or mercury_xacro_frames.yaml) with an "
                              "imu pose. Picked configs are remembered.");
  robot_row->addWidget(robot_selector_, 1);
  robot_remove_ = new QPushButton("Forget");
  robot_remove_->setToolTip("Remove this robot config from the list");
  robot_row->addWidget(robot_remove_);
  column->addLayout(robot_row);
  robot_detail_ = toneLabel(QString(), "muted");
  robot_detail_->setObjectName("robotDetail");
  robot_detail_->setMinimumHeight(2 * robot_detail_->fontMetrics().lineSpacing() + 2);
  column->addWidget(robot_detail_);
  connect(robot_selector_, qOverload<int>(&QComboBox::activated), this, [this](int index) {
    const QString data = robot_selector_->itemData(index).toString();
    if (data != "add") {
      useRobotConfig(data);
      return;
    }
    const QString start = mount_ ? QFileInfo(mount_->path).absolutePath() : QDir::homePath();
    const QString path =
        QFileDialog::getOpenFileName(this, "Robot config with an imu pose", start, "YAML (*.yaml *.yml)");
    if (path.isEmpty() || !useRobotConfig(path)) refreshRobotChoices();
  });
  connect(robot_remove_, &QPushButton::clicked, this, [this] {
    if (!mount_) return;
    QSettings settings;
    QStringList saved = settings.value("calibration/robots").toStringList();
    saved.removeAll(mount_->path);
    settings.setValue("calibration/robots", saved);
    useRobotConfig(QString());
  });

  column->addSpacing(8);
  column->addWidget(sectionTitle("Steps to run"));
  const char* descriptions[] = {
      "30 s hands-off hold. Corrects gyro bias (register 84).",
      "Nine tilted poses. Corrects accelerometer bias (register 25).",
      "One slow, level turn. Corrects in-plane hard iron (register 23).",
      "10 s hold as mounted. Report only; changes nothing."};
  for (int step = 0; step < kCalibrationStepCount; ++step) {
    auto* row = new QHBoxLayout;
    selected_[step] = new QCheckBox(calibrationStepName(static_cast<CalibrationStep>(step)));
    selected_[step]->setChecked(true);
    selected_[step]->setMinimumWidth(150);
    selected_[step]->setFont(strongFont(14));
    row->addWidget(selected_[step]);
    row->addWidget(toneLabel(descriptions[step], "muted"), 1);
    column->addLayout(row);
    connect(selected_[step], &QCheckBox::toggled, this, [this] { refreshNavigation(); });
  }
  column->addWidget(toneLabel("A step you skip leaves its register as it is.", "muted"));
  column->addStretch();
  column->setContentsMargins(0, 0, 8, 0);
  layout->addWidget(scrollingColumn(column), 5);
  return page;
}

QWidget* CalibrationWizard::buildStepPage(Page index, const QString& title, const QStringList&, QWidget* gauge) {
  auto* page = new QWidget;
  page->setObjectName("wizardPage");
  auto& p = stepPage(index);
  p.page = page;
  auto* layout = new QHBoxLayout(page);
  layout->setContentsMargins(24, 20, 24, 20);
  layout->setSpacing(28);

  // left: drawing, gauge, readings
  auto* left = new QVBoxLayout;
  left->setSpacing(14);
  p.illustration = new GuideIllustration;
  left->addWidget(p.illustration, 1);
  p.live = new MetricTable;
  const bool big_gauge = index == Accelerometer || index == Magnetometer;
  if (big_gauge) {
    auto* live_row = new QHBoxLayout;
    live_row->setSpacing(20);
    live_row->addWidget(gauge, 1);
    auto* live_column = new QVBoxLayout;
    live_column->addWidget(p.live);
    live_column->addStretch();
    live_row->addLayout(live_column);
    left->addLayout(live_row, 1);
  } else {
    left->addWidget(gauge);
    left->addWidget(p.live);
  }
  layout->addLayout(left, 5);

  auto* right = new QVBoxLayout;
  right->setSpacing(8);
  right->addWidget(pageTitle(title));
  p.instructions = instructionList(instructionsFor(index));
  right->addWidget(p.instructions);
  right->addWidget(sectionTitle("Capture"));
  p.status = toneLabel("Not started.", "muted");
  p.status->setObjectName("captureStatus");
  right->addWidget(p.status);
  p.progress = new QProgressBar;
  p.progress->setRange(0, 1000);
  p.progress->setTextVisible(false);
  p.progress->setFixedHeight(8);
  right->addWidget(p.progress);
  auto* actions = new QHBoxLayout;
  p.action = new QPushButton("Start capture");
  p.action->setObjectName("captureButton");
  p.action->setProperty("role", "primary");
  actions->addWidget(p.action);
  connect(p.action, &QPushButton::clicked, this, [this] {
    if (phase_ == Phase::Settling || phase_ == Phase::Capturing) cancelCapture(); else startCapture();
  });
  if (big_gauge) {
    auto* finish = new QPushButton("Finish with what I have");
    finish->setObjectName("finishButton");
    finish->setEnabled(false);
    actions->addWidget(finish);
    connect(finish, &QPushButton::clicked, this, &CalibrationWizard::finishCapture);
  }
  actions->addStretch();
  right->addLayout(actions);
  right->addSpacing(8);
  p.checks_title = sectionTitle("Checks");
  p.checks_title->setToolTip("Whether the capture can be trusted. Each has a limit; all OK means the result is sound.");
  right->addWidget(p.checks_title);
  p.verdict = toneLabel("Capture to see the result.", "muted");
  right->addWidget(p.verdict);
  p.result = new MetricTable;
  p.result->setObjectName("resultChecks");
  right->addWidget(p.result);
  p.advice = toneLabel(QString(), "warn");
  p.advice->setVisible(false);
  right->addWidget(p.advice);
  right->addSpacing(8);
  p.output_title = sectionTitle("Result");
  right->addWidget(p.output_title);
  p.output = new QFrame;
  p.output->setObjectName("resultCard");
  auto* card = new QVBoxLayout(p.output);
  card->setContentsMargins(14, 10, 14, 12);
  card->setSpacing(6);
  p.output_heading = new QLabel;
  p.output_heading->setFont(strongFont(14));
  p.output_heading->setWordWrap(true);
  card->addWidget(p.output_heading);
  p.output_values = new QLabel;
  p.output_values->setObjectName("resultValues");
  p.output_values->setTextFormat(Qt::RichText);
  p.output_values->setTextInteractionFlags(Qt::TextSelectableByMouse);
  card->addWidget(p.output_values);
  p.output_note = toneLabel(QString(), "muted");
  p.output_note->setTextFormat(Qt::RichText);
  card->addWidget(p.output_note);
  p.output_action = new QPushButton;
  p.output_action->hide();
  auto* action_row = new QHBoxLayout;
  action_row->addWidget(p.output_action);
  action_row->addStretch();
  card->addLayout(action_row);
  right->addWidget(p.output);
  right->addStretch();
  right->setContentsMargins(0, 0, 8, 0);
  layout->addWidget(scrollingColumn(right), 6);
  return page;
}

QWidget* CalibrationWizard::buildReviewPage() {
  auto* page = new QWidget;
  page->setObjectName("wizardPage");
  auto& p = stepPage(Review);
  p.page = page;
  auto* layout = new QVBoxLayout(page);
  layout->setContentsMargins(32, 24, 32, 24);
  layout->setSpacing(12);
  layout->addWidget(pageTitle("Review and stage"));
  layout->addWidget(paragraph(
      "Staging copies each passed result into the register workspace. The sensor is not changed until you "
      "apply the staged registers, and the change is lost at power-off until you save to flash.", "muted"));
  review_table_ = new MetricTable;
  review_table_->setObjectName("reviewTable");
  review_table_->setColumns({"Step", "Result", "Applies to"}, {0.2, 0.62, 0.18});
  layout->addWidget(review_table_);
  auto* actions = new QHBoxLayout;
  stage_button_ = new QPushButton("Stage passed results");
  stage_button_->setObjectName("stageButton");
  stage_button_->setProperty("role", "primary");
  auto* export_button = new QPushButton("Export report…");
  actions->addWidget(stage_button_);
  actions->addWidget(export_button);
  actions->addStretch();
  layout->addLayout(actions);
  review_note_ = toneLabel(QString(), "muted");
  layout->addWidget(review_note_);
  layout->addSpacing(10);
  layout->addWidget(sectionTitle("Then, in the main window"));
  layout->addWidget(instructionList(
      {"Staged registers are marked <b>Staged</b> in the register list. Select one to compare its device and "
       "staged values.",
       "Press <b>Apply</b> on the command bar. Every write is read back from the sensor.",
       "Press <b>Save to flash</b> to keep the values after power-off. On the robot this needs maintenance "
       "mode, and the IMU output pauses briefly while the sensor resets."}));
  layout->addStretch();
  connect(stage_button_, &QPushButton::clicked, this, [this] {
    for (auto step : {CalibrationStep::Gyro, CalibrationStep::Accelerometer, CalibrationStep::Magnetometer})
      if (stepState(step) == StepState::Passed) emit stageRequested(step);
  });
  connect(export_button, &QPushButton::clicked, this, &CalibrationWizard::exportRequested);
  return page;
}

int CalibrationWizard::currentPage() const { return pages_ ? pages_->currentIndex() : 0; }

bool CalibrationWizard::stepSelected(int page) const {
  if (!isCapturePage(page)) return true;
  const auto* box = selected_[page - Gyro];
  return !box || box->isChecked();
}

int CalibrationWizard::nextPage(int from, int direction) const {
  int page = from + direction;
  while (page > Prepare && page < Review && !stepSelected(page)) page += direction;
  return std::clamp(page, static_cast<int>(Prepare), static_cast<int>(Review));
}

void CalibrationWizard::goToPage(int page) {
  if (page != currentPage()) cancelCapture();
  pages_->setCurrentIndex(page);
  for (int step = 0; step < kCalibrationStepCount; ++step)
    if (!stepSelected(step + Gyro) && states_[step] == StepState::NotRun) states_[step] = StepState::Skipped;
  if (isCapturePage(page)) {
    auto& p = stepPage(static_cast<Page>(page));
    switch (page) {
      case Gyro:
        p.illustration->setMode(GuideIllustration::Mode::Hold);
        p.illustration->setCaption("Set it down and step away", "Any orientation, on something solid.");
        break;
      case Gravity:
        p.illustration->setMode(GuideIllustration::Mode::Hold);
        p.illustration->setCaption("As mounted, hands off", "The robot in its normal resting attitude.");
        break;
      case Accelerometer: p.illustration->setMode(GuideIllustration::Mode::Pose); break;
      case Magnetometer:
        p.illustration->setMode(GuideIllustration::Mode::Turn);
        p.illustration->setCaption("Turn slowly, keep it level", "About the vertical, through a full circle.");
        break;
      default: break;
    }
    refreshResult(static_cast<Page>(page));
  }
  if (page == Review) refreshReview();
  refreshLive();
  refreshNavigation();
}

Sample CalibrationWizard::toSample(double t, const Eigen::Vector3d& value, double temperature) {
  return {t, value, temperature};
}

void CalibrationWizard::addSample(const Telemetry& sample) {
  // time holds by sample stamps so transport delay can't shorten one
  double t = sample.time_s > 0.0 ? sample.time_s : clock_.elapsed() / 1000.0;
  if (t < now_) {
    monitor_.reset();
    temperatures_.clear();
  }
  now_ = t;
  ++data_samples_;
  if (data_clock_.elapsed() >= 1000) {
    data_rate_hz_ = 1000.0 * data_samples_ / data_clock_.elapsed();
    data_samples_ = 0;
    data_clock_.restart();
  } else if (data_rate_hz_ == 0.0) {
    data_rate_hz_ = -1.0;  // receiving, rate not measured yet
  }
  last_sample_ = sample;
  temperatures_.push_back({t, sample.temperature_c});
  while (temperatures_.size() > 2 && t - temperatures_.front().first > kTemperatureWindowS) temperatures_.pop_front();
  monitor_.add(t, {sample.gx, sample.gy, sample.gz}, {sample.ax, sample.ay, sample.az});
  // checks run on the sample clock
  if (currentPage() == Prepare && (t < last_prepare_refresh_ || t - last_prepare_refresh_ >= 0.5)) {
    last_prepare_refresh_ = t;
    refreshPrepare();
  }

  if (phase_ == Phase::Settling || phase_ == Phase::Capturing) {
    switch (currentPage()) {
      case Gyro: handleHoldSample(sample, t, kGyroHoldS); break;
      case Gravity: handleHoldSample(sample, t, kGravityHoldS); break;
      case Accelerometer: handlePoseSample(sample, t); break;
      case Magnetometer: handleTurnSample(sample, t); break;
      default: break;
    }
  }
  refreshLive();
}

void CalibrationWizard::handleHoldSample(const Telemetry& sample, double t, double duration_s) {
  const bool gyro = currentPage() == Gyro;
  auto& buffer = gyro ? session_->gyro : session_->gravity;
  if (phase_ == Phase::Settling) {
    if (!monitor_.still()) return;
    buffer.clear();
    capture_start_ = t;
    min_temperature_ = max_temperature_ = sample.temperature_c;
    setPhase(Phase::Capturing, "Holding. Keep hands off until the bar is full.");
    return;
  }
  if (!monitor_.still()) {
    buffer.clear();
    ++restarts_;
    setPhase(Phase::Settling,
             "Movement detected, so the hold started over. It resumes on its own once the sensor settles.",
             "warn");
    return;
  }
  const Eigen::Vector3d value = gyro ? Eigen::Vector3d(sample.gx, sample.gy, sample.gz)
                                     : Eigen::Vector3d(sample.ax, sample.ay, sample.az);
  buffer.push_back(toSample(t - capture_start_, value, sample.temperature_c));
  min_temperature_ = std::min(min_temperature_, sample.temperature_c);
  max_temperature_ = std::max(max_temperature_, sample.temperature_c);
  if (t - capture_start_ >= duration_s && buffer.size() >= policy_.minimum_samples) finishCapture();
}

void CalibrationWizard::handlePoseSample(const Telemetry& sample, double t) {
  const Eigen::Vector3d accel(sample.ax, sample.ay, sample.az);
  const Eigen::Vector3d direction = gravityDirection(accel);
  int nearest = -1, shallow = -1;
  double best = kPoseToleranceDeg;
  for (std::size_t i = 0; i < poses_.size(); ++i) {
    if (captured_[i]) continue;
    const double angle = angleBetweenDeg(direction, poses_[i].direction());
    if (angle >= best) continue;
    if (poseReached(poses_[i], direction, kPoseToleranceDeg)) {
      best = angle;
      nearest = static_cast<int>(i);
    } else {
      shallow = static_cast<int>(i);
    }
  }
  if (nearest < 0 || !monitor_.still()) {
    if (holding_ >= 0) hold_buffer_.clear();
    holding_ = -1;
    if (nearest < 0 && shallow >= 0)
      setPhase(Phase::Capturing,
               QString("Almost: tip further, to at least %1°.")
                   .arg(poses_[shallow].tilt_deg - kPoseShortfallDeg, 0, 'f', 0), "warn");
    else if (nearest < 0)
      setPhase(Phase::Capturing, QString("Tip the %1 until the dot sits inside a ring.")
                                     .arg(frame_.robot ? "robot" : "sensor"));
    else
      setPhase(Phase::Capturing, QString("In the %1 ring. Let go and hold still.").arg(
                                     QString::fromStdString(poses_[nearest].label)));
    return;
  }
  if (holding_ != nearest) {
    holding_ = nearest;
    hold_start_ = t;
    hold_buffer_.clear();
    setPhase(Phase::Capturing, QString("Holding %1…").arg(QString::fromStdString(poses_[nearest].label)));
  }
  hold_buffer_.push_back(toSample(t, accel, sample.temperature_c));
  const std::size_t per_pose =
      std::max<std::size_t>(10, policy_.minimum_samples / poses_.size() + 1);
  if (t - hold_start_ >= kPoseHoldS && hold_buffer_.size() >= per_pose) {
    session_->accelerometer.insert(session_->accelerometer.end(), hold_buffer_.begin(), hold_buffer_.end());
    captured_[holding_] = true;
    const int done = static_cast<int>(std::count(captured_.begin(), captured_.end(), true));
    setPhase(Phase::Capturing, QString("Captured %1 (%2 of %3). Move to the next ring.")
                                   .arg(QString::fromStdString(poses_[holding_].label))
                                   .arg(done).arg(poses_.size()), "ok");
    holding_ = -1;
    hold_buffer_.clear();
    emit sessionChanged();
    if (done == static_cast<int>(poses_.size())) finishCapture();
  }
}

void CalibrationWizard::handleTurnSample(const Telemetry& sample, double t) {
  const Eigen::Vector3d accel(sample.ax, sample.ay, sample.az);
  session_->magnetometer.push_back(
      toSample(t - capture_start_, {sample.mx, sample.my, sample.mz}, sample.temperature_c));
  session_->magnetometer_acceleration.push_back(toSample(t - capture_start_, accel, sample.temperature_c));
  coverage_.add(t, {sample.gx, sample.gy, sample.gz}, accel);
  const double remaining = std::max(0.0, policy_.mag_min_coverage_deg - coverage_.coverageDeg());
  if (coverage_.tiltVariationDeg() > policy_.mag_max_tilt_stddev_deg)
    setPhase(Phase::Capturing, "Tilted more than the limit. Keep the sensor level while you turn.", "warn");
  else if (std::abs(coverage_.yawRateDegS()) > kMaxTurnRateDegS)
    setPhase(Phase::Capturing, "Turning fast. Slow down to about 12° per second.", "warn");
  else if (remaining > 0)
    setPhase(Phase::Capturing, QString("Keep turning slowly, %1° to go.").arg(remaining, 0, 'f', 0));
  else
    setPhase(Phase::Capturing, "Enough heading covered. Finish the circle, or press Finish.", "ok");
  if (coverage_.coverageDeg() >= 360.0 && session_->magnetometer.size() >= policy_.minimum_samples)
    finishCapture();
}

void CalibrationWizard::startCapture() {
  const int page = currentPage();
  if (!isCapturePage(page)) return;
  if (!last_sample_) {
    setPhase(Phase::Idle, "No calibration data is arriving. Connect to the sensor first.", "error");
    return;
  }
  const auto step = stepFor(static_cast<Page>(page));
  session_->clear(step);
  states_[static_cast<int>(step)] = StepState::Capturing;
  restarts_ = 0;
  const double t = now_;
  capture_start_ = t;
  switch (page) {
    case Gyro:
    case Gravity:
      setPhase(Phase::Settling, "Waiting for the sensor to settle…");
      break;
    case Accelerometer:
      captured_.assign(poses_.size(), false);
      holding_ = -1;
      hold_buffer_.clear();
      setPhase(Phase::Capturing, QString("Tip the %1 until the dot sits inside a ring.")
                                     .arg(frame_.robot ? "robot" : "sensor"));
      break;
    case Magnetometer:
      // start at zero; a still sensor's mean rate is gyro bias and would creep the heading
      coverage_.reset(gravityDirection(monitor_.hasData() ? monitor_.meanAccel()
                                                          : Eigen::Vector3d(last_sample_->ax, last_sample_->ay,
                                                                            last_sample_->az)),
                      monitor_.still() ? monitor_.meanGyro() : Eigen::Vector3d::Zero());
      coverage_.add(t, Eigen::Vector3d::Zero(), {last_sample_->ax, last_sample_->ay, last_sample_->az});
      setPhase(Phase::Capturing, "Turn slowly, keeping the sensor level.");
      break;
    default: break;
  }
  refreshResult(static_cast<Page>(page));
  refreshNavigation();
  emit sessionChanged();
}

void CalibrationWizard::cancelCapture() {
  if (phase_ != Phase::Settling && phase_ != Phase::Capturing) return;
  const int page = currentPage();
  if (isCapturePage(page)) {
    const auto step = stepFor(static_cast<Page>(page));
    session_->clear(step);
    states_[static_cast<int>(step)] = StepState::NotRun;
  }
  holding_ = -1;
  hold_buffer_.clear();
  setPhase(Phase::Idle, "Capture canceled. Nothing was kept.");
  refreshNavigation();
  emit sessionChanged();
}

void CalibrationWizard::finishCapture() {
  const int page = currentPage();
  if (!isCapturePage(page)) return;
  const auto step = stepFor(static_cast<Page>(page));
  holding_ = -1;
  hold_buffer_.clear();
  session_->evaluate(step, policy_);
  const bool passed = session_->quality(step).passed;
  states_[static_cast<int>(step)] = passed ? StepState::Passed : StepState::Failed;
  setPhase(Phase::Done, passed ? "Done. Continue to the next step." : "Done, but the result did not pass.",
           passed ? "ok" : "error");
  refreshResult(static_cast<Page>(page));
  refreshNavigation();
  emit sessionChanged();
}

void CalibrationWizard::setPhase(Phase phase, const QString& message, const QString& tone) {
  phase_ = phase;
  const int page = currentPage();
  if (!isCapturePage(page)) return;
  auto& p = stepPage(static_cast<Page>(page));
  if (p.status->text() != message) p.status->setText(message);
  if (p.status->property("tone").toString() != tone) setTone(p.status, tone);
  const bool running = phase == Phase::Settling || phase == Phase::Capturing;
  p.action->setText(running ? "Cancel" : phase == Phase::Done ? "Capture again" : "Start capture");
  const QString role = running ? "" : "primary";
  if (p.action->property("role").toString() != role) {
    p.action->setProperty("role", role);
    p.action->style()->unpolish(p.action);
    p.action->style()->polish(p.action);
  }
}

void CalibrationWizard::refreshLive() {
  const int page = currentPage();
  if (!isCapturePage(page)) return;
  auto& p = stepPage(static_cast<Page>(page));
  const bool running = phase_ == Phase::Settling || phase_ == Phase::Capturing;
  const double t = now_;
  std::vector<MetricRow> rows;
  double progress = phase_ == Phase::Done ? 1.0 : 0.0;
  std::optional<Eigen::Vector3d> live;
  if (last_sample_) live = gravityDirection({last_sample_->ax, last_sample_->ay, last_sample_->az});

  if (page == Gyro || page == Gravity) {
    (page == Gyro ? gyro_meter_ : gravity_meter_)->setReading(monitor_);
    const double duration = page == Gyro ? kGyroHoldS : kGravityHoldS;
    const auto count = session_->sampleCount(stepFor(static_cast<Page>(page)));
    if (phase_ == Phase::Capturing) progress = std::min(1.0, (t - capture_start_) / duration);
    rows.push_back({"Samples", QString::number(count), QString("≥ %1").arg(policy_.minimum_samples),
                    count >= policy_.minimum_samples ? Level::Ok : Level::Neutral, ""});
    if (page == Gyro) {
      const double span = phase_ == Phase::Capturing ? max_temperature_ - min_temperature_ : 0.0;
      const bool span_ok = span <= policy_.gyro_max_temperature_span_c;
      rows.push_back({"Temp. change", number(span, 2) + " °C",
                      "≤ " + number(policy_.gyro_max_temperature_span_c, 1) + " °C",
                      within(span_ok), phase_ == Phase::Capturing ? (span_ok ? "OK" : "Over") : ""});
    }
    rows.push_back({"Hold restarts", QString::number(restarts_), "", Level::Neutral, ""});
  } else if (page == Accelerometer) {
    pose_meter_->setReading(monitor_);
    const int done = static_cast<int>(std::count(captured_.begin(), captured_.end(), true));
    // suggest the nearest unfilled ring
    int suggested = -1;
    double nearest = 1e9;
    for (std::size_t i = 0; i < captured_.size(); ++i) {
      if (captured_[i]) continue;
      const double angle = live ? angleBetweenDeg(*live, poses_[i].direction()) : static_cast<double>(i);
      if (angle < nearest) {
        nearest = angle;
        suggested = static_cast<int>(i);
      }
    }
    const double hold = holding_ >= 0 ? std::min(1.0, (t - hold_start_) / kPoseHoldS) : 0.0;
    tilt_disc_->setState(captured_, running ? holding_ : -1, running ? suggested : -1, hold);
    tilt_disc_->setLive(live);
    progress = std::max(progress, static_cast<double>(done) / poses_.size());
    const int shown = holding_ >= 0 ? holding_ : suggested >= 0 ? suggested : 0;
    p.illustration->setTarget(poses_[shown].direction());
    p.illustration->setLive(live);
    p.illustration->setCaption(
        QString("%1: %2").arg(holding_ >= 0 ? "Holding" : "Next").arg(QString::fromStdString(poses_[shown].label)),
        shown == 0 ? (frame_.robot ? QString("Robot level, as it normally sits.")
                                   : QString("Sensor flat, right side up."))
                   : QString("Tip about %1° so that %2 is the lowest.").arg(kPoseTiltDeg, 0, 'f', 0)
                         .arg(frame_.robot ? "part of the robot" : "end"));
    Eigen::Vector3d low = Eigen::Vector3d::Zero(), high = low;
    if (!session_->accelerometer.empty()) low = high = session_->accelerometer.front().value;
    for (const auto& s : session_->accelerometer) {
      low = low.cwiseMin(s.value);
      high = high.cwiseMax(s.value);
    }
    rows.push_back({"Poses", QString("%1 of %2").arg(done).arg(poses_.size()), "",
                    done == static_cast<int>(poses_.size()) ? Level::Ok : Level::Neutral, ""});
    const char* axes[] = {"X span", "Y span", "Z span"};
    for (int axis = 0; axis < 3; ++axis) {
      const double span = high(axis) - low(axis);
      const bool ok = span >= policy_.accel_min_axis_span_m_s2;
      rows.push_back({axes[axis], number(span, 2) + " m/s²", "≥ " + number(policy_.accel_min_axis_span_m_s2, 1),
                      within(ok), done ? (ok ? "OK" : "Short") : ""});
    }
    if (auto* finish = p.page->findChild<QPushButton*>("finishButton"))
      finish->setEnabled(running && session_->accelerometer.size() >= policy_.minimum_samples);
  } else if (page == Magnetometer) {
    coverage_ring_->setCoverage(coverage_, policy_.mag_min_coverage_deg, running || phase_ == Phase::Done);
    if (running) progress = std::min(1.0, coverage_.coverageDeg() / 360.0);
    const bool coverage_ok = coverage_.coverageDeg() >= policy_.mag_min_coverage_deg;
    const bool tilt_ok = coverage_.tiltVariationDeg() <= policy_.mag_max_tilt_stddev_deg;
    const bool rate_ok = std::abs(coverage_.yawRateDegS()) <= kMaxTurnRateDegS;
    const bool show = running || phase_ == Phase::Done;
    rows.push_back({"Heading covered", number(show ? coverage_.coverageDeg() : 0.0, 0) + "°",
                    "≥ " + number(policy_.mag_min_coverage_deg, 0) + "°", within(coverage_ok),
                    show ? (coverage_ok ? "OK" : "") : ""});
    rows.push_back({"Largest tilt", number(show ? coverage_.tiltVariationDeg() : 0.0, 1) + "°",
                    "≤ " + number(policy_.mag_max_tilt_stddev_deg, 0) + "°", within(tilt_ok),
                    show ? (tilt_ok ? "OK" : "Over") : ""});
    rows.push_back({"Turn rate", number(show ? std::abs(coverage_.yawRateDegS()) : 0.0, 0) + "°/s",
                    "≤ " + number(kMaxTurnRateDegS, 0) + "°/s", within(rate_ok),
                    running ? (rate_ok ? "OK" : "Fast") : ""});
    rows.push_back({"Samples", QString::number(session_->magnetometer.size()),
                    QString("≥ %1").arg(policy_.minimum_samples), Level::Neutral, ""});
    if (auto* finish = p.page->findChild<QPushButton*>("finishButton"))
      finish->setEnabled(running && coverage_ok && session_->magnetometer.size() >= policy_.minimum_samples);
  }
  p.live->setRows(rows);
  p.progress->setValue(static_cast<int>(progress * 1000));
}

std::vector<MetricRow> CalibrationWizard::resultRows(CalibrationStep step) const {
  std::vector<MetricRow> rows;
  auto check = [](bool ok) { return ok ? QString("OK") : QString("Fail"); };
  switch (step) {
    case CalibrationStep::Gyro: {
      const auto& r = session_->gyro_result;
      const bool rms = r.quality.rms <= policy_.gyro_max_rms_rad_s;
      const bool temp = r.temperature_span_c <= policy_.gyro_max_temperature_span_c;
      rows.push_back({"Noise RMS", QString::number(r.quality.rms, 'g', 3) + " rad/s",
                      "≤ " + QString::number(policy_.gyro_max_rms_rad_s, 'g', 3), within(rms), check(rms)});
      rows.push_back({"Temperature change", number(r.temperature_span_c, 2) + " °C",
                      "≤ " + number(policy_.gyro_max_temperature_span_c, 1), within(temp), check(temp)});
      break;
    }
    case CalibrationStep::Accelerometer: {
      const auto& r = session_->accelerometer_result;
      const bool rms = r.quality.rms <= policy_.accel_max_rms_m_s2;
      const bool condition = r.quality.condition <= policy_.accel_max_condition;
      rows.push_back({"Fit RMS", number(r.quality.rms, 4) + " m/s²", "≤ " + number(policy_.accel_max_rms_m_s2, 2),
                      within(rms), check(rms)});
      rows.push_back({"Pose spread (condition)", number(r.quality.condition, 0),
                      "≤ " + number(policy_.accel_max_condition, 0), within(condition), check(condition)});
      const char* axes[] = {"X span", "Y span", "Z span"};
      for (int axis = 0; axis < 3; ++axis) {
        const bool ok = r.axis_span(axis) >= policy_.accel_min_axis_span_m_s2;
        rows.push_back({axes[axis], number(r.axis_span(axis), 2) + " m/s²",
                        "≥ " + number(policy_.accel_min_axis_span_m_s2, 1), within(ok), check(ok)});
      }
      break;
    }
    case CalibrationStep::Magnetometer: {
      const auto& r = session_->magnetometer_result;
      const double fraction = r.radius > 0 ? r.quality.rms / r.radius : 0.0;
      const bool rms = fraction <= policy_.mag_max_rms_fraction;
      const bool coverage = r.coverage_deg >= policy_.mag_min_coverage_deg;
      rows.push_back({"Circle fit RMS", number(100 * fraction, 1) + " % of field",
                      "≤ " + number(100 * policy_.mag_max_rms_fraction, 0) + " %", within(rms), check(rms)});
      rows.push_back({"Heading coverage", number(r.coverage_deg, 0) + "°",
                      "≥ " + number(policy_.mag_min_coverage_deg, 0) + "°", within(coverage), check(coverage)});
      break;
    }
    case CalibrationStep::Gravity: {
      const auto& r = session_->gravity_report;
      const bool rms = r.quality.rms <= policy_.accel_max_rms_m_s2;
      rows.push_back({"Noise RMS", number(r.quality.rms, 4) + " m/s²", "≤ " + number(policy_.accel_max_rms_m_s2, 2),
                      within(rms), check(rms)});
      break;
    }
  }
  return rows;
}

QString CalibrationWizard::adviceFor(CalibrationStep step) const {
  const auto& quality = session_->quality(step);
  if (quality.passed) return {};
  if (QString::fromStdString(quality.summary).startsWith("Insufficient") ||
      session_->sampleCount(step) < policy_.minimum_samples)
    return "Not enough samples were captured. Capture again and let it run to the end.";
  switch (step) {
    case CalibrationStep::Gyro: {
      const auto& r = session_->gyro_result;
      if (r.quality.rms > policy_.gyro_max_rms_rad_s)
        return "Something moved during the hold, such as footsteps, a pump, or a cable being touched. Put the "
               "sensor on a solid surface, step back and capture again.";
      if (r.temperature_span_c > policy_.gyro_max_temperature_span_c)
        return QString("The temperature changed %1 °C during the hold. Leave the sensor powered for about ten "
                       "minutes so it reaches a steady temperature, then capture again.")
            .arg(r.temperature_span_c, 0, 'f', 2);
      return "The bias drifted during the hold, usually from warming up or a slow lean. Wait for a steady "
             "temperature and capture again.";
    }
    case CalibrationStep::Accelerometer: {
      const auto& r = session_->accelerometer_result;
      int short_axis = 0;
      r.axis_span.minCoeff(&short_axis);
      if (r.axis_span(short_axis) < policy_.accel_min_axis_span_m_s2)
        return QString("The %1 span was only %2 m/s². Tip further, a full 45° at the end and corner poses, so "
                       "every axis sees gravity change.")
            .arg(QString("XYZ"[short_axis])).arg(r.axis_span(short_axis), 0, 'f', 2);
      if (r.quality.rms > policy_.accel_max_rms_m_s2)
        return "Readings varied within poses. Let the sensor settle fully before each ring, and make sure it is "
               "not rocking on its support.";
      return "The poses were too similar to separate bias from gravity. Use all nine rings.";
    }
    case CalibrationStep::Magnetometer: {
      const auto& r = session_->magnetometer_result;
      if (r.coverage_deg < policy_.mag_min_coverage_deg)
        return QString("Only %1° of heading was covered. Turn the sensor through a complete circle.")
            .arg(r.coverage_deg, 0, 'f', 0);
      if (r.radius > 0 && r.quality.rms > policy_.mag_max_rms_fraction * r.radius)
        return "The field did not trace a clean circle. Keep steel, magnets, phones and power cables away, keep "
               "it level, and turn slowly in one direction.";
      return "The sensor tilted too much while turning. Keep it level, for example on a cart or a turntable.";
    }
    case CalibrationStep::Gravity:
      return "Readings were not steady. Let the robot settle and keep hands off, then capture again.";
  }
  return {};
}

void CalibrationWizard::refreshResult(Page page) {
  auto& p = stepPage(page);
  const auto step = stepFor(page);
  const bool evaluated = session_->evaluated[static_cast<int>(step)];
  if (!evaluated) {
    p.result->setRows({});
    p.verdict->setText(phase_ == Phase::Idle ? "Capture to see the checks." : "Capturing…");
    setTone(p.verdict, "muted");
    p.advice->setVisible(false);
  } else {
    const bool passed = session_->quality(step).passed;
    p.verdict->setText(passed ? "All within limits, so the result below can be trusted."
                              : "Outside a limit, so the result below is not trustworthy and cannot be staged.");
    setTone(p.verdict, passed ? "ok" : "error");
    p.result->setRows(resultRows(step));
    const QString advice = adviceFor(step);
    p.advice->setText(advice.isEmpty() ? QString() : "What to try:  " + advice);
    p.advice->setVisible(!advice.isEmpty());
  }
  refreshOutput(page);
}

void CalibrationWizard::refreshOutput(Page page) {
  auto& p = stepPage(page);
  const auto step = stepFor(page);
  const int index = static_cast<int>(step);
  const bool evaluated = session_->evaluated[index];
  const bool passed = evaluated && session_->quality(step).passed;
  p.output->setVisible(evaluated);
  p.output_title->setVisible(evaluated);
  p.output_action->hide();
  if (!evaluated) return;
  const auto& t = theme();
  auto table = [&](const QStringList& heads, const std::vector<std::pair<QString, QStringList>>& rows) {
    QString html = "<table cellspacing='0' cellpadding='0'><tr><td></td>";
    for (int i = 0; i < heads.size(); ++i)
      html += QString("<td align='right' style='padding:0 0 3px 18px; color:%1'><b>%2</b></td>")
                  .arg((heads.size() == 3 && heads[0] == "X" ? axisColor(i) : t.muted).name(), heads[i]);
    html += "</tr>";
    for (const auto& [label, values] : rows) {
      html += QString("<tr><td style='padding:2px 0; color:%1'>%2</td>").arg(t.muted.name(), label);
      for (const auto& value : values)
        html += QString("<td align='right' style='padding:2px 0 2px 18px; font-family:\"Barlow TF Semi Condensed\";"
                        " font-weight:bold; font-size:16px'>%1</td>").arg(value);
      html += "</tr>";
    }
    return html + "</table>";
  };
  auto figures = [](const Eigen::Vector3d& v, int decimals, double scale = 1.0) {
    QStringList list;
    for (int i = 0; i < 3; ++i) list << QString("%1%2").arg(v(i) * scale >= 0 ? "+" : "−")
                                            .arg(std::abs(v(i) * scale), 0, 'f', decimals);
    return list;
  };
  auto sigma3 = [](const Eigen::Matrix3d& covariance) {
    return Eigen::Vector3d(3 * covariance.diagonal().cwiseMax(0.0).cwiseSqrt());
  };
  auto plusMinus = [&](const Eigen::Vector3d& v, int decimals, double scale = 1.0) {
    QStringList list;
    for (const auto& value : figures(v, decimals, scale)) list << "±" + value.mid(1);
    return list;
  };

  QString heading, note;
  QString values;
  if (step == CalibrationStep::Gyro) {
    const auto& r = session_->gyro_result;
    heading = "Gyro bias measured while still";
    values = table({"X", "Y", "Z"}, {{"rad/s", figures(r.residual_bias, 6)},
                                     {"3σ", plusMinus(sigma3(r.covariance), 6)},
                                     {"°/s", figures(r.residual_bias, 3, 180.0 / M_PI)}});
    note = "This is the rate the gyro read while nothing turned, and staging subtracts it. Its size is not graded: "
           "a large bias measured cleanly is still right. Judge it by the checks above, and by 3σ being small "
           "beside the bias.";
  } else if (step == CalibrationStep::Accelerometer) {
    const auto& r = session_->accelerometer_result;
    heading = "Accelerometer bias from the sphere fit";
    values = table({"X", "Y", "Z"}, {{"m/s²", figures(r.bias, 4)}, {"3σ", plusMinus(sigma3(r.covariance), 4)}});
    note = QString("Fitted gravity %1 ± %2 m/s². It should be close to local gravity (about 9.80 m/s²); a fit far "
                   "from it means the poses or the existing compensation are off.")
               .arg(r.measured_gravity, 0, 'f', 3).arg(r.gravity_stddev, 0, 'f', 3);
  } else if (step == CalibrationStep::Magnetometer) {
    const auto& r = session_->magnetometer_result;
    heading = "Hard-iron offset in the turning plane";
    values = table({"X", "Y", "Z"}, {{"G", figures(r.in_plane_bias, 4)}});
    note = QString("Field radius %1 G; the offset is %2% of it. Only the part in the plane you turned through is "
                   "measured, so the axis you turned about keeps its present bias.")
               .arg(r.radius, 0, 'f', 3).arg(r.radius > 0 ? 100 * r.in_plane_bias.norm() / r.radius : 0.0, 0, 'f', 0);
  } else {
    const auto& r = session_->gravity_report;
    heading = "Sensor attitude against gravity";
    const Eigen::Vector3d down = r.gravity_vector.normalized();
    std::vector<std::pair<QString, QStringList>> rows{
        {"Sensor (VN-100 convention)", {signedDeg(r.roll_deg), signedDeg(r.pitch_deg)}}};
    QStringList notes{QString("±%1° (1σ). Roll is positive with the sensor's right (+Y) side down, pitch positive "
                              "with its +X end up, as the VN-100 reports them. |g| %2 m/s².")
                          .arg(r.tilt_stddev_deg, 0, 'f', 2).arg(r.magnitude, 0, 'f', 3)};
    if (mount_ && down.allFinite()) {
      const auto check = checkMount(down, mount_->sensor_to_base);
      rows.push_back({QString("%1 base_link (ROS) *").arg(mount_->name), {signedDeg(check.roll_deg),
                                                                          signedDeg(check.pitch_deg)}});
      const Eigen::Vector3d rpy = check.corrected_rpy;
      const QString pose = QString("pose: [%1, %2, %3, %4, %5, %6]")
                               .arg(mount_->xyz.x()).arg(mount_->xyz.y()).arg(mount_->xyz.z())
                               .arg(rpy.x(), 0, 'f', 4).arg(rpy.y(), 0, 'f', 4).arg(rpy.z(), 0, 'f', 4);
      // robot figures only mean something if the robot was level
      notes.prepend(QString("<span style='color:%1'><b>* Only valid if the robot was level during the capture.</b> "
                            "On a sloped floor, or with the robot not resting in its normal attitude, these "
                            "figures and the pose below measure that tilt, not the mount.</span>")
                        .arg(theme().warn.name()));
      notes << QString("With %1's IMU rotation the robot reads %2° from level (ROS: roll positive right side "
                       "down, pitch positive nose down).").arg(mount_->name).arg(check.error_deg, 0, 'f', 2);
      notes << QString("* The IMU rotation in %1 that would make it read level (yaw unchanged, it cannot be "
                       "seen from gravity):<br><code>%2</code>")
                   .arg(QFileInfo(mount_->path).fileName(), pose);
      const QString copied = pose + "  # from the gravity report; valid only if the robot was level during capture";
      p.output_action->setText("Copy pose line");
      p.output_action->disconnect();
      connect(p.output_action, &QPushButton::clicked, this,
              [copied] { QApplication::clipboard()->setText(copied); });
      p.output_action->show();
    } else {
      notes << "Pick a robot config on the Prepare page to get the robot's roll and pitch and a corrected IMU pose.";
    }
    values = table({"Roll", "Pitch"}, rows);
    note = notes.join("<br>");
  }

  const int id = compensationRegister(step);
  if (id >= 0) {
    const auto delivery = deliveryState(*session_, step, device_registers_, staged_registers_);
    QString where;
    if (!passed) {
      where = "Not stageable: the checks did not pass.";
    } else if (delivery == Delivery::Staged) {
      where = QString("Staged into register %1. Apply it in the main window to write it.").arg(id);
    } else if (delivery == Delivery::Applied) {
      where = QString("On the sensor in register %1. Save to flash in the main window to keep it.").arg(id);
    } else {
      const auto proposal = proposeCompensation(*session_, step, device_registers_);
      if (!proposal.error.isEmpty()) {
        where = proposal.error + ".";
      } else {
        const int decimals = step == CalibrationStep::Gyro ? 6 : 5;
        where = QString("Staging sets register %1's bias from %2 to %3.")
                    .arg(id).arg("[" + figures(proposal.old_bias, decimals).join(", ") + "]")
                    .arg("[" + figures(proposal.new_bias, decimals).join(", ") + "]");
      }
    }
    note += "<br><br>" + where;
  } else {
    note += "<br><br>Report only: nothing is written to the sensor.";
  }
  p.output_heading->setText(heading);
  p.output_values->setText(values);
  p.output_note->setText(note);
}

void CalibrationWizard::refreshPrepare() {
  if (!data_chip_) return;
  const bool receiving = last_sample_ && data_rate_hz_ != 0.0;
  if (receiving) {
    data_chip_->setState(data_rate_hz_ > 0 ? QString("%1 Hz").arg(data_rate_hz_, 0, 'f', 0) : QString("Live"),
                         Level::Ok);
    data_detail_->setText("Samples are arriving.");
  } else {
    data_chip_->setState(connected_ ? "No data" : "Offline", connected_ ? Level::Warn : Level::Error);
    data_detail_->setText(connected_ ? "Connected, but no calibration samples yet."
                                     : "Connect to the robot or sensor in the main window.");
  }
  if (registers_missing_.isEmpty()) {
    registers_chip_->setState("Read", Level::Ok);
    registers_detail_->setText("23, 25, 26, 43 and 84 are known, so results can be staged.");
  } else {
    registers_chip_->setState("Missing", Level::Warn);
    registers_detail_->setText("Not read yet: " + registers_missing_ + ". Needed to stage results.");
  }
  reread_->setEnabled(connected_);
  const QString limit = QString("Gyro step: under %1 °C/min.").arg(kSteadyRateCMin + 0.05, 0, 'f', 1);
  if (temperatures_.size() < 2 || temperatures_.back().first - temperatures_.front().first < 30.0) {
    temperature_known_ = false;
    temperature_candidate_since_ = -1.0;
    warmup_chip_->setState(receiving ? "Watching" : "Unknown", Level::Neutral);
    warmup_detail_->setText("Measuring the trend for 30 s.\n" + limit);
    return;
  }
  // least-squares slope
  const double t0 = temperatures_.front().first;
  double n = 0, st = 0, stt = 0, sy = 0, sty = 0;
  for (const auto& [time, value] : temperatures_) {
    const double x = time - t0;
    n += 1, st += x, stt += x * x, sy += value, sty += x * value;
  }
  const double denominator = n * stt - st * st;
  const double rate = denominator > 0 ? (n * sty - st * sy) / denominator * 60.0 : 0.0;
  // hysteresis on rate and time
  const bool candidate = temperature_known_ ? (temperature_steady_ ? std::abs(rate) < kChangingRateCMin
                                                                   : std::abs(rate) < kSteadyRateCMin)
                                            : std::abs(rate) < kSteadyRateCMin;
  if (!temperature_known_ || candidate != temperature_steady_) {
    if (temperature_candidate_since_ < 0) temperature_candidate_since_ = now_;
    if (!temperature_known_ || now_ - temperature_candidate_since_ >= kTemperatureHoldS) {
      temperature_steady_ = candidate;
      temperature_known_ = true;
      temperature_candidate_since_ = -1.0;
    }
  } else {
    temperature_candidate_since_ = -1.0;
  }
  warmup_chip_->setState(temperature_steady_ ? "Steady" : "Changing", temperature_steady_ ? Level::Ok : Level::Warn);
  const QString trend = std::abs(rate) < 0.05 ? QString("holding")
                        : QString("%1 %2 °C/min").arg(rate > 0 ? "rising" : "falling")
                              .arg(std::abs(rate), 0, 'f', 2);
  warmup_detail_->setText(QString("%1 °C, %2.\n").arg(temperatures_.back().second, 0, 'f', 1).arg(trend) + limit);
}

void CalibrationWizard::refreshReview() {
  if (!review_table_) return;
  std::vector<MetricRow> rows;
  const char* targets[] = {"Register 84", "Register 25", "Register 23", "Report only"};
  bool stageable = false;
  for (int step = 0; step < kCalibrationStepCount; ++step) {
    const auto s = static_cast<CalibrationStep>(step);
    QString verdict;
    Level level = Level::Neutral;
    switch (stepState(s)) {
      case StepState::NotRun: verdict = "Not run"; break;
      case StepState::Skipped: verdict = "Skipped"; break;
      case StepState::Capturing: verdict = "Running"; level = Level::Info; break;
      case StepState::Passed: verdict = "Passed"; level = Level::Ok; break;
      case StepState::Failed: verdict = "Failed"; level = Level::Error; break;
      case StepState::Staged: verdict = "Staged"; level = Level::Change; break;
      case StepState::Applied: verdict = "On sensor"; level = Level::Info; break;
    }
    stageable |= step < 3 && stepState(s) == StepState::Passed;
    rows.push_back({calibrationStepName(s), calibrationHeadline(*session_, s), targets[step], level, verdict});
  }
  review_table_->setRows(rows);
  stage_button_->setEnabled(stageable);
  QStringList notes;
  for (int step = 0; step < kCalibrationStepCount; ++step)
    if (!stage_messages_[step].isEmpty()) notes << stage_messages_[step];
  review_note_->setText(notes.join("\n"));
}

void CalibrationWizard::refreshNavigation() {
  const int page = currentPage();
  std::vector<WizardStepper::Entry> entries;
  entries.push_back({"Prepare", "Checks and steps", Level::Neutral, true});
  for (int step = 0; step < kCalibrationStepCount; ++step) {
    WizardStepper::Entry entry;
    entry.title = calibrationStepName(static_cast<CalibrationStep>(step));
    entry.enabled = stepSelected(step + Gyro);
    switch (stepState(static_cast<CalibrationStep>(step))) {
      case StepState::NotRun: entry.state = entry.enabled ? "Not run" : "Skipped"; break;
      case StepState::Skipped: entry.state = "Skipped"; break;
      case StepState::Capturing: entry.state = "Capturing"; break;
      case StepState::Passed: entry.state = "Passed"; entry.level = Level::Ok; break;
      case StepState::Failed: entry.state = "Did not pass"; entry.level = Level::Error; break;
      case StepState::Staged: entry.state = "Staged"; entry.level = Level::Change; break;
      case StepState::Applied: entry.state = "On sensor"; entry.level = Level::Ok; break;
    }
    entries.push_back(entry);
  }
  entries.push_back({"Review", "Stage and export", Level::Neutral, true});
  stepper_->setEntries(entries, page);

  const QString names[] = {"Prepare", "Gyro bias", "Accelerometer", "Magnetometer", "Gravity report", "Review"};
  header_step_->setText(QString("STEP %1 OF %2 · %3").arg(page + 1).arg(PageCount).arg(names[page].toUpper()));
  const bool receiving = last_sample_ && data_rate_hz_ != 0.0;
  header_chip_->setState(receiving ? (data_rate_hz_ > 0 ? QString("Live %1 Hz").arg(data_rate_hz_, 0, 'f', 0)
                                                        : QString("Live"))
                                   : connected_ ? QString("No data") : QString("Offline"),
                         receiving ? Level::Ok : connected_ ? Level::Warn : Level::Error);
  back_->setEnabled(page > Prepare);
  next_->setText(page == Review ? "Close" : page == Prepare ? "Begin" : "Next");
  skip_->setVisible(isCapturePage(page));
  const bool running = phase_ == Phase::Settling || phase_ == Phase::Capturing;
  skip_->setEnabled(running || isCapturePage(page));
  if (page == Review) refreshReview();
}

void CalibrationWizard::setConnected(bool connected) {
  connected_ = connected;
  if (!connected) {
    last_sample_.reset();
    data_rate_hz_ = 0.0;
    cancelCapture();
  }
  refreshPrepare();
  refreshNavigation();
}

void CalibrationWizard::setRegistersMissing(const QString& missing) {
  registers_missing_ = missing;
  refreshPrepare();
}

void CalibrationWizard::setStageOutcome(CalibrationStep step, bool, const QString& message) {
  const int index = static_cast<int>(step);
  stage_messages_[index] = QString("%1: %2").arg(calibrationStepName(step), message);
  refreshReview();
  refreshNavigation();
}

void CalibrationWizard::setRegisterContext(const QMap<int, QString>& device, const QMap<int, QString>& staged) {
  device_registers_ = device;
  staged_registers_ = staged;
  if (isCapturePage(currentPage())) refreshOutput(static_cast<Page>(currentPage()));
  refreshReview();
  refreshNavigation();
}

CalibrationWizard::StepState CalibrationWizard::stepState(CalibrationStep step) const {
  const StepState state = states_[static_cast<int>(step)];
  if (state != StepState::Passed) return state;
  // read off the registers each time
  switch (deliveryState(*session_, step, device_registers_, staged_registers_)) {
    case Delivery::Staged: return StepState::Staged;
    case Delivery::Applied: return StepState::Applied;
    case Delivery::None: break;
  }
  return state;
}

void CalibrationWizard::showEvent(QShowEvent* event) {
  QDialog::showEvent(event);
  emit captureRequested(true);
  emit registersRequested();
}

void CalibrationWizard::hideEvent(QHideEvent* event) {
  cancelCapture();
  emit captureRequested(false);
  QDialog::hideEvent(event);
}

}  // namespace riptide_imu
