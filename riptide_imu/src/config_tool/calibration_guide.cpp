#include "riptide_imu/config_tool/calibration_guide.hpp"

#include <Eigen/Geometry>
#include <QJsonArray>
#include <QString>
#include <algorithm>
#include <cmath>

#include "riptide_imu/config_tool/configuration.hpp"
#include "riptide_imu/config_tool/robot_mount.hpp"

namespace riptide_imu {
namespace {

QJsonArray vec(const Eigen::Vector3d& v) { return QJsonArray{v.x(), v.y(), v.z()}; }

QJsonArray matrix(const Eigen::Matrix3d& value) {
  QJsonArray rows;
  for (int row = 0; row < 3; ++row) rows.push_back(QJsonArray{value(row, 0), value(row, 1), value(row, 2)});
  return rows;
}

double wrapDeg(double angle) {
  angle = std::fmod(angle, 360.0);
  return angle < 0.0 ? angle + 360.0 : angle;
}

}  // namespace

const char* calibrationStepName(CalibrationStep step) {
  switch (step) {
    case CalibrationStep::Gyro: return "Gyro bias";
    case CalibrationStep::Accelerometer: return "Accelerometer";
    case CalibrationStep::Magnetometer: return "Magnetometer";
    case CalibrationStep::Gravity: return "Gravity report";
  }
  return "";
}

void CalibrationSession::clear(CalibrationStep step) {
  switch (step) {
    case CalibrationStep::Gyro: gyro.clear(); gyro_result = {}; break;
    case CalibrationStep::Accelerometer: accelerometer.clear(); accelerometer_result = {}; break;
    case CalibrationStep::Magnetometer:
      magnetometer.clear();
      magnetometer_acceleration.clear();
      magnetometer_result = {};
      break;
    case CalibrationStep::Gravity: gravity.clear(); gravity_report = {}; break;
  }
  evaluated[static_cast<int>(step)] = false;
  delivered[static_cast<int>(step)].clear();
}

std::size_t CalibrationSession::sampleCount(CalibrationStep step) const {
  switch (step) {
    case CalibrationStep::Gyro: return gyro.size();
    case CalibrationStep::Accelerometer: return accelerometer.size();
    case CalibrationStep::Magnetometer: return magnetometer.size();
    case CalibrationStep::Gravity: return gravity.size();
  }
  return 0;
}

bool CalibrationSession::evaluate(CalibrationStep step, const CalibrationPolicy& policy) {
  if (sampleCount(step) == 0 &&
      !(step == CalibrationStep::Magnetometer && !magnetometer_acceleration.empty()))
    return false;
  switch (step) {
    case CalibrationStep::Gyro: gyro_result = estimateGyroBias(gyro, policy); break;
    case CalibrationStep::Accelerometer:
      accelerometer_result = estimateAccelerometerBias(accelerometer, policy);
      break;
    case CalibrationStep::Magnetometer:
      magnetometer_result = estimatePlanarMagBias(magnetometer, magnetometer_acceleration, policy);
      break;
    case CalibrationStep::Gravity: gravity_report = makeGravityReport(gravity, policy); break;
  }
  evaluated[static_cast<int>(step)] = true;
  return true;
}

const Quality& CalibrationSession::quality(CalibrationStep step) const {
  switch (step) {
    case CalibrationStep::Gyro: return gyro_result.quality;
    case CalibrationStep::Accelerometer: return accelerometer_result.quality;
    case CalibrationStep::Magnetometer: return magnetometer_result.quality;
    case CalibrationStep::Gravity: break;
  }
  return gravity_report.quality;
}

QJsonObject calibrationReport(CalibrationSession& session, const CalibrationPolicy& policy) {
  // Always report the samples currently in memory, so a step that was captured but never evaluated does not
  // export default result objects.
  for (int step = 0; step < kCalibrationStepCount; ++step)
    session.evaluate(static_cast<CalibrationStep>(step), policy);
  const auto& gyro = session.gyro_result;
  const auto& accel = session.accelerometer_result;
  const auto& mag = session.magnetometer_result;
  const auto& gravity = session.gravity_report;
  QJsonObject report{
      {"schema_version", 1},
      {"warning", "Gravity-derived tilt is not sensor-to-robot mounting rotation; yaw is unobservable."},
      {"gravity_sign", "gravity_vector_sensor = -mean_stationary_specific_force"},
      {"frame", "VectorNav configured sensor output frame"}};
  report["gyro"] = QJsonObject{
      {"evaluated", !gyro.quality.summary.empty()},
      {"sample_count", static_cast<int>(session.gyro.size())},
      {"passed", gyro.quality.passed},
      {"residual_bias_rad_s", vec(gyro.residual_bias)},
      {"bias_covariance_rad2_s2", matrix(gyro.covariance)},
      {"quality_rms_rad_s", gyro.quality.rms},
      {"temperature_span_c", gyro.temperature_span_c},
      {"quality", QString::fromStdString(gyro.quality.summary)}};
  report["accelerometer"] = QJsonObject{
      {"evaluated", !accel.quality.summary.empty()},
      {"sample_count", static_cast<int>(session.accelerometer.size())},
      {"passed", accel.quality.passed},
      {"bias_m_s2", vec(accel.bias)},
      {"bias_covariance_m2_s4", matrix(accel.covariance)},
      {"axis_span_m_s2", vec(accel.axis_span)},
      {"measured_magnitude_m_s2", accel.measured_gravity},
      {"magnitude_stddev_m_s2", accel.gravity_stddev},
      {"quality_rms_m_s2", accel.quality.rms},
      {"condition", accel.quality.condition},
      {"quality", QString::fromStdString(accel.quality.summary)}};
  report["magnetometer"] = QJsonObject{
      {"evaluated", !mag.quality.summary.empty()},
      {"sample_count", static_cast<int>(session.magnetometer.size())},
      {"paired_gravity_sample_count", static_cast<int>(session.magnetometer_acceleration.size())},
      {"passed", mag.quality.passed},
      {"in_plane_bias_gauss", vec(mag.in_plane_bias)},
      {"coverage_deg", mag.coverage_deg},
      {"quality", QString::fromStdString(mag.quality.summary)}};
  report["gravity_report"] = QJsonObject{
      {"evaluated", !gravity.quality.summary.empty()},
      {"sample_count", static_cast<int>(session.gravity.size())},
      {"passed", gravity.quality.passed},
      {"specific_force_mean_m_s2", vec(gravity.specific_force_mean)},
      {"gravity_vector_m_s2", vec(gravity.gravity_vector)},
      {"mean_covariance_m2_s4", matrix(gravity.covariance)},
      {"magnitude_m_s2", gravity.magnitude},
      {"magnitude_stddev_m_s2", gravity.magnitude_stddev},
      {"tilt_from_sensor_z_deg", gravity.tilt_from_sensor_z_deg},
      {"tilt_stddev_deg", gravity.tilt_stddev_deg},
      {"roll_deg", gravity.roll_deg},
      {"pitch_deg", gravity.pitch_deg},
      {"quality_rms_m_s2", gravity.quality.rms},
      {"quality", QString::fromStdString(gravity.quality.summary)}};
  report["policy"] = QJsonObject{
      {"minimum_samples", static_cast<int>(policy.minimum_samples)},
      {"gyro_max_rms_rad_s", policy.gyro_max_rms_rad_s},
      {"gyro_max_temperature_span_c", policy.gyro_max_temperature_span_c},
      {"gyro_max_pose_delta_rad_s", policy.gyro_max_pose_delta_rad_s},
      {"accelerometer_max_rms_m_s2", policy.accel_max_rms_m_s2},
      {"accelerometer_max_condition", policy.accel_max_condition},
      {"accelerometer_min_axis_span_m_s2", policy.accel_min_axis_span_m_s2},
      {"magnetometer_max_rms_fraction", policy.mag_max_rms_fraction},
      {"magnetometer_min_coverage_deg", policy.mag_min_coverage_deg},
      {"magnetometer_max_tilt_stddev_deg", policy.mag_max_tilt_stddev_deg}};
  return report;
}

Eigen::Vector3d gravityDirection(const Eigen::Vector3d& specific_force) {
  const double norm = specific_force.norm();
  if (!(norm > 1e-9) || !specific_force.allFinite()) return Eigen::Vector3d::UnitZ();
  return -specific_force / norm;
}

double angleBetweenDeg(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
  const double denominator = a.norm() * b.norm();
  if (!(denominator > 0.0)) return 0.0;
  return std::acos(std::clamp(a.dot(b) / denominator, -1.0, 1.0)) * 180.0 / M_PI;
}

StillnessMonitor::StillnessMonitor(double window_s, double gyro_limit_rad_s, double accel_limit_m_s2)
    : window_s_(window_s), gyro_limit_(gyro_limit_rad_s), accel_limit_(accel_limit_m_s2) {}

void StillnessMonitor::reset() {
  samples_.clear();
  gyro_spread_ = accel_spread_ = 0.0;
  gyro_mean_ = accel_mean_ = Eigen::Vector3d::Zero();
}

void StillnessMonitor::add(double time_s, const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel) {
  if (!gyro.allFinite() || !accel.allFinite()) return;
  if (!samples_.empty() && time_s < samples_.back().time_s) samples_.clear();
  samples_.push_back({time_s, gyro, accel});
  while (samples_.size() > 2 && time_s - samples_.front().time_s > window_s_) samples_.pop_front();
  gyro_mean_ = accel_mean_ = Eigen::Vector3d::Zero();
  for (const auto& entry : samples_) {
    gyro_mean_ += entry.gyro;
    accel_mean_ += entry.accel;
  }
  gyro_mean_ /= samples_.size();
  accel_mean_ /= samples_.size();
  double gyro_squared = 0.0, accel_squared = 0.0;
  for (const auto& entry : samples_) {
    gyro_squared += (entry.gyro - gyro_mean_).squaredNorm();
    accel_squared += (entry.accel - accel_mean_).squaredNorm();
  }
  gyro_spread_ = std::sqrt(gyro_squared / samples_.size());
  accel_spread_ = std::sqrt(accel_squared / samples_.size());
}

bool StillnessMonitor::still() const {
  if (samples_.size() < 3) return false;
  // The window must be mostly full, or a capture could start from one lucky reading.
  if (samples_.back().time_s - samples_.front().time_s < 0.6 * window_s_) return false;
  return gyro_spread_ <= gyro_limit_ && accel_spread_ <= accel_limit_;
}

GuideFrame sensorGuideFrame() { return {}; }

GuideFrame robotGuideFrame(const QString& name, const Eigen::Matrix3d& sensor_to_base) {
  GuideFrame frame;
  // base_link is X forward, Y left, Z up; the display wants Y right and Z down
  frame.display_from_sensor = Eigen::Vector3d(1, -1, -1).asDiagonal() * sensor_to_base;
  frame.robot = true;
  frame.name = name;
  frame.ends = {"Front", "Right", "Back", "Left"};
  return frame;
}

Eigen::Vector3d TiltTarget::direction() const {
  const double tilt = tilt_deg * M_PI / 180.0;
  const double azimuth = azimuth_deg * M_PI / 180.0;
  return sensor_from_display *
         Eigen::Vector3d(std::sin(tilt) * std::cos(azimuth), std::sin(tilt) * std::sin(azimuth), std::cos(tilt));
}

Eigen::Vector3d TiltTarget::level() const { return sensor_from_display * Eigen::Vector3d::UnitZ(); }

std::vector<TiltTarget> accelerometerPoses(double tilt_deg, const GuideFrame& frame) {
  const char* sensor_labels[] = {"Flat", "+X end down", "+X/+Y corner down", "+Y side down", "-X/+Y corner down",
                                 "-X end down", "-X/-Y corner down", "-Y side down", "+X/-Y corner down"};
  const char* robot_labels[] = {"Level", "Front down", "Front-right corner down", "Right side down",
                                "Back-right corner down", "Back down", "Back-left corner down", "Left side down",
                                "Front-left corner down"};
  const Eigen::Matrix3d sensor_from_display = frame.display_from_sensor.transpose();
  std::vector<TiltTarget> poses;
  for (int i = 0; i < 9; ++i)
    poses.push_back({frame.robot ? robot_labels[i] : sensor_labels[i], i == 0 ? 0.0 : tilt_deg,
                     i == 0 ? 0.0 : 45.0 * (i - 1), sensor_from_display});
  return poses;
}

bool poseReached(const TiltTarget& target, const Eigen::Vector3d& gravity_in_body, double tolerance_deg) {
  if (angleBetweenDeg(gravity_in_body, target.direction()) > tolerance_deg) return false;
  if (target.tilt_deg <= 0.0) return true;
  return angleBetweenDeg(gravity_in_body, target.level()) >= target.tilt_deg - kPoseShortfallDeg;
}

void HeadingCoverage::reset(const Eigen::Vector3d& gravity_direction, const Eigen::Vector3d& gyro_bias) {
  up_ = gravity_direction.normalized();
  bias_ = gyro_bias.allFinite() ? gyro_bias : Eigen::Vector3d::Zero();
  last_time_ = -1.0;
  heading_deg_ = low_deg_ = high_deg_ = 0.0;
  max_tilt_deg_ = 0.0;
  yaw_rate_deg_s_ = 0.0;
}

void HeadingCoverage::add(double time_s, const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel) {
  if (!gyro.allFinite() || !accel.allFinite()) return;
  max_tilt_deg_ = std::max(max_tilt_deg_, angleBetweenDeg(gravityDirection(accel), up_));
  if (last_time_ < 0.0 || time_s <= last_time_) {
    last_time_ = time_s;
    return;
  }
  const double dt = std::min(time_s - last_time_, 0.25);
  last_time_ = time_s;
  // Rotation about the gravity axis; the sign only flips the drawing direction.
  yaw_rate_deg_s_ = (gyro - bias_).dot(up_) * 180.0 / M_PI;
  heading_deg_ += yaw_rate_deg_s_ * dt;
  low_deg_ = std::min(low_deg_, heading_deg_);
  high_deg_ = std::max(high_deg_, heading_deg_);
}

double HeadingCoverage::headingDeg() const { return wrapDeg(heading_deg_); }

std::array<bool, HeadingCoverage::kSectors> HeadingCoverage::sectors() const {
  std::array<bool, kSectors> covered{};
  const double span = 360.0 / kSectors;
  if (coverageDeg() >= 360.0) {
    covered.fill(true);
    return covered;
  }
  for (int i = 0; i < kSectors; ++i) {
    // a sector counts once the turn has passed its middle, either way round
    const double middle = (i + 0.5) * span;
    for (double candidate : {middle, middle - 360.0})
      if (candidate >= low_deg_ && candidate <= high_deg_) covered[i] = true;
  }
  return covered;
}

Eigen::Vector2d levelRollPitchDeg(const Eigen::Vector3d& down) {
  const Eigen::Vector3d g = down.normalized();
  return Eigen::Vector2d(std::atan2(g.y(), g.z()), std::atan2(-g.x(), std::hypot(g.y(), g.z()))) * 180.0 / M_PI;
}

MountCheck checkMount(const Eigen::Vector3d& sensor_down, const Eigen::Matrix3d& sensor_to_base) {
  MountCheck check;
  check.base_down = (sensor_to_base * sensor_down).normalized();
  // In base_link (Z up) a level robot sees gravity along -Z: roll = atan2(-g_y, -g_z), pitch = asin(g_x).
  check.roll_deg = std::atan2(-check.base_down.y(), -check.base_down.z()) * 180.0 / M_PI;
  check.pitch_deg = std::asin(std::clamp(check.base_down.x(), -1.0, 1.0)) * 180.0 / M_PI;
  check.error_deg = angleBetweenDeg(check.base_down, -Eigen::Vector3d::UnitZ());
  const Eigen::Matrix3d level =
      Eigen::Quaterniond::FromTwoVectors(check.base_down, -Eigen::Vector3d::UnitZ()).toRotationMatrix();
  check.corrected_rpy = matrixToRpy(level * sensor_to_base);
  return check;
}

bool parseCompensation(const QString& text, Eigen::Matrix3d* matrix, Eigen::Vector3d* bias) {
  const auto fields = text.split(';');
  if (fields.size() != 12) return false;
  bool ok = true;
  for (int i = 0; i < 12; ++i) {
    bool part = false;
    const double value = fields[i].toDouble(&part);
    if (i < 9) (*matrix)(i / 3, i % 3) = value; else (*bias)(i - 9) = value;
    ok &= part;
  }
  return ok;
}

QString formatCompensation(const Eigen::Matrix3d& matrix, const Eigen::Vector3d& bias) {
  QStringList fields;
  for (int row = 0; row < 3; ++row)
    for (int column = 0; column < 3; ++column)
      fields << QString::number(static_cast<float>(matrix(row, column)), 'g', 9);
  for (int i = 0; i < 3; ++i) fields << QString::number(static_cast<float>(bias(i)), 'g', 9);
  return fields.join(';');
}

int compensationRegister(CalibrationStep step) {
  switch (step) {
    case CalibrationStep::Gyro: return 84;
    case CalibrationStep::Accelerometer: return 25;
    case CalibrationStep::Magnetometer: return 23;
    case CalibrationStep::Gravity: break;
  }
  return -1;
}

CompensationProposal proposeCompensation(const CalibrationSession& session, CalibrationStep step,
                                         const QMap<int, QString>& device) {
  CompensationProposal proposal;
  proposal.register_id = compensationRegister(step);
  if (proposal.register_id < 0) {
    proposal.error = "The gravity report has no register update by design";
    return proposal;
  }
  if (!session.evaluated[static_cast<int>(step)] || !session.quality(step).passed) {
    proposal.error = "Quality gate failed; result cannot be staged";
    return proposal;
  }
  Eigen::Matrix3d matrix;
  Eigen::Matrix3d rotation;
  Eigen::Vector3d unused;
  const bool have_register = parseCompensation(device.value(proposal.register_id), &matrix, &proposal.old_bias);
  const bool have_rotation = parseCompensation(device.value(26) + ";0;0;0", &rotation, &unused);
  if (!have_register) proposal.unread << proposal.register_id;
  if (!have_rotation) proposal.unread << 26;
  Eigen::Vector3d residual;
  if (step == CalibrationStep::Gyro) {
    const auto fields = device.value(43).split(';');
    Eigen::Vector3d startup;
    bool ok = fields.size() == 3;
    for (int i = 0; i < 3 && ok; ++i) startup(i) = fields[i].toDouble(&ok);
    if (!ok) {
      proposal.unread << 43;
    } else if (startup.norm() > 1e-7) {
      proposal.error = "Gyro staging blocked: a nonzero startup filter bias (register 43) would double-correct "
                       "angular rate";
      return proposal;
    }
    residual = session.gyro_result.residual_bias;
  } else if (step == CalibrationStep::Accelerometer) {
    residual = session.accelerometer_result.bias;
  } else {
    residual = session.magnetometer_result.in_plane_bias;
  }
  if (!proposal.unread.isEmpty()) {
    QStringList ids;
    for (int id : proposal.unread) ids << QString::number(id);
    proposal.error = "Read register " + ids.join(", ") + " first";
    return proposal;
  }
  try {
    proposal.new_bias = composeCompensationBias(matrix, rotation, proposal.old_bias, residual);
  } catch (const std::exception& exception) {
    proposal.error = QString("Register %1 not staged: %2").arg(proposal.register_id).arg(exception.what());
    return proposal;
  }
  proposal.values = formatCompensation(matrix, proposal.new_bias);
  return proposal;
}

Delivery deliveryState(const CalibrationSession& session, CalibrationStep step, const QMap<int, QString>& device,
                       const QMap<int, QString>& staged) {
  const QString& delivered = session.delivered[static_cast<int>(step)];
  const int id = compensationRegister(step);
  if (delivered.isEmpty() || id < 0) return Delivery::None;
  if (staged.contains(id)) return registerValuesMatch(delivered, staged.value(id)) ? Delivery::Staged : Delivery::None;
  return device.contains(id) && registerValuesMatch(delivered, device.value(id)) ? Delivery::Applied
                                                                                 : Delivery::None;
}

}  // namespace riptide_imu
