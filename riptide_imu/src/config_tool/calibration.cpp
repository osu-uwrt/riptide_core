#include "riptide_imu/config_tool/calibration.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>

namespace riptide_imu {
namespace {

double median(std::vector<double> values) {
  if (values.empty()) return 0.0;
  const auto middle = values.begin() + values.size() / 2;
  std::nth_element(values.begin(), middle, values.end());
  double result = *middle;
  if (values.size() % 2 == 0) {
    const auto lower = std::max_element(values.begin(), middle);
    result = 0.5 * (result + *lower);
  }
  return result;
}

Eigen::Vector3d robustCenter(const std::vector<Eigen::Vector3d>& values) {
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  for (int axis = 0; axis < 3; ++axis) {
    std::vector<double> component;
    component.reserve(values.size());
    for (const auto& value : values) component.push_back(value(axis));
    center(axis) = median(std::move(component));
  }
  return center;
}

Eigen::Matrix3d covariance(const std::vector<Eigen::Vector3d>& values,
                           const Eigen::Vector3d& center) {
  Eigen::Matrix3d result = Eigen::Matrix3d::Zero();
  if (values.size() < 2) return result;
  for (const auto& value : values) {
    const Eigen::Vector3d delta = value - center;
    result.noalias() += delta * delta.transpose();
  }
  return result / static_cast<double>(values.size() - 1);
}

std::vector<Eigen::Vector3d> vectors(const std::vector<Sample>& samples) {
  std::vector<Eigen::Vector3d> result;
  result.reserve(samples.size());
  for (const auto& sample : samples) {
    if (sample.value.allFinite()) result.push_back(sample.value);
  }
  return result;
}

}  // namespace

CalibrationPolicy loadCalibrationPolicy(const std::string& path) {
  QFile file(QString::fromStdString(path));
  if (!file.open(QIODevice::ReadOnly)) throw std::runtime_error(file.errorString().toStdString());
  QJsonParseError parse_error;
  const QJsonDocument document = QJsonDocument::fromJson(file.readAll(), &parse_error);
  if (parse_error.error != QJsonParseError::NoError || !document.isObject())
    throw std::runtime_error("Invalid calibration policy JSON: " + parse_error.errorString().toStdString());
  const QJsonObject object = document.object();
  if (object.value("schema_version").toInt() != 1)
    throw std::runtime_error("Unsupported calibration policy schema version");
  CalibrationPolicy policy;
  policy.minimum_samples = object.value("minimum_samples").toInt(policy.minimum_samples);
  policy.gyro_blocks = object.value("gyro_blocks").toInt(policy.gyro_blocks);
  policy.gyro_max_rms_rad_s = object.value("gyro_max_rms_rad_s").toDouble(policy.gyro_max_rms_rad_s);
  policy.gyro_max_temperature_span_c = object.value("gyro_max_temperature_span_c").toDouble(policy.gyro_max_temperature_span_c);
  policy.gyro_max_pose_delta_rad_s = object.value("gyro_max_pose_delta_rad_s").toDouble(policy.gyro_max_pose_delta_rad_s);
  policy.accel_max_rms_m_s2 = object.value("accelerometer_max_rms_m_s2").toDouble(policy.accel_max_rms_m_s2);
  policy.accel_max_condition = object.value("accelerometer_max_condition").toDouble(policy.accel_max_condition);
  policy.accel_min_axis_span_m_s2 = object.value("accelerometer_min_axis_span_m_s2").toDouble(policy.accel_min_axis_span_m_s2);
  policy.mag_max_rms_fraction = object.value("magnetometer_max_rms_fraction").toDouble(policy.mag_max_rms_fraction);
  policy.mag_min_coverage_deg = object.value("magnetometer_min_coverage_deg").toDouble(policy.mag_min_coverage_deg);
  policy.mag_max_tilt_stddev_deg = object.value("magnetometer_max_tilt_stddev_deg").toDouble(policy.mag_max_tilt_stddev_deg);
  if (policy.minimum_samples < 2 || policy.gyro_blocks < 2 ||
      policy.gyro_max_rms_rad_s <= 0.0 || policy.accel_max_rms_m_s2 <= 0.0 ||
      policy.mag_min_coverage_deg <= 0.0 || policy.mag_min_coverage_deg > 360.0)
    throw std::runtime_error("Calibration policy contains invalid limits");
  return policy;
}

GyroResult estimateGyroBias(const std::vector<Sample>& samples,
                            const CalibrationPolicy& policy) {
  GyroResult result;
  if (samples.size() < policy.minimum_samples || policy.gyro_blocks < 2) {
    result.quality.summary = "Insufficient stationary gyro samples";
    return result;
  }
  const std::size_t block_size = samples.size() / policy.gyro_blocks;
  if (block_size < 2) {
    result.quality.summary = "Insufficient samples per correlation block";
    return result;
  }
  std::vector<Eigen::Vector3d> block_means;
  for (std::size_t block = 0; block < policy.gyro_blocks; ++block) {
    const std::size_t begin = block * block_size;
    const std::size_t end = block + 1 == policy.gyro_blocks
                                ? samples.size()
                                : begin + block_size;
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (std::size_t i = begin; i < end; ++i) mean += samples[i].value;
    block_means.push_back(mean / static_cast<double>(end - begin));
  }
  result.residual_bias = robustCenter(block_means);
  result.covariance = covariance(block_means, result.residual_bias) /
                      static_cast<double>(block_means.size());

  double squared = 0.0;
  double max_block_delta = 0.0;
  double min_temp = std::numeric_limits<double>::infinity();
  double max_temp = -std::numeric_limits<double>::infinity();
  for (const auto& sample : samples) {
    squared += (sample.value - result.residual_bias).squaredNorm();
    min_temp = std::min(min_temp, sample.temperature_c);
    max_temp = std::max(max_temp, sample.temperature_c);
  }
  for (const auto& mean : block_means)
    max_block_delta = std::max(max_block_delta,
                               (mean - result.residual_bias).norm());
  result.quality.rms = std::sqrt(squared / samples.size());
  result.temperature_span_c = max_temp - min_temp;
  result.quality.passed =
      result.quality.rms <= policy.gyro_max_rms_rad_s &&
      result.temperature_span_c <= policy.gyro_max_temperature_span_c &&
      max_block_delta <= policy.gyro_max_pose_delta_rad_s;
  std::ostringstream message;
  message << (result.quality.passed ? "Passed" : "Failed")
          << ": RMS " << result.quality.rms << " rad/s, temperature span "
          << result.temperature_span_c << " C, block drift "
          << max_block_delta << " rad/s. Earth rotation remains in the uncertainty budget.";
  result.quality.summary = message.str();
  return result;
}

AccelerometerResult estimateAccelerometerBias(
    const std::vector<Sample>& samples, const CalibrationPolicy& policy) {
  AccelerometerResult result;
  const auto values = vectors(samples);
  if (values.size() < policy.minimum_samples) {
    result.quality.summary = "Insufficient accelerometer samples";
    return result;
  }

  Eigen::MatrixXd design(values.size(), 4);
  Eigen::VectorXd rhs(values.size());
  for (std::size_t i = 0; i < values.size(); ++i) {
    design.row(i) << 2.0 * values[i].transpose(), 1.0;
    rhs(i) = values[i].squaredNorm();
  }
  Eigen::VectorXd weights = Eigen::VectorXd::Ones(values.size());
  Eigen::Vector4d solution = Eigen::Vector4d::Zero();
  for (int iteration = 0; iteration < 12; ++iteration) {
    const Eigen::MatrixXd weighted = weights.asDiagonal() * design;
    solution = weighted.colPivHouseholderQr().solve(weights.asDiagonal() * rhs);
    const Eigen::VectorXd residual = design * solution - rhs;
    std::vector<double> absolute;
    absolute.reserve(residual.size());
    for (int i = 0; i < residual.size(); ++i) absolute.push_back(std::abs(residual(i)));
    const double scale = std::max(1e-9, 1.4826 * median(std::move(absolute)));
    for (int i = 0; i < residual.size(); ++i) {
      const double normalized = std::abs(residual(i)) / (1.345 * scale);
      weights(i) = normalized <= 1.0 ? 1.0 : 1.0 / normalized;
    }
  }
  result.bias = solution.head<3>();
  const double radius_squared = solution(3) + result.bias.squaredNorm();
  if (!(radius_squared > 0.0) || !std::isfinite(radius_squared)) {
    result.quality.summary = "Sphere fit produced a nonphysical gravity magnitude";
    return result;
  }
  result.measured_gravity = std::sqrt(radius_squared);

  double squared = 0.0;
  Eigen::MatrixXd jacobian(values.size(), 4);
  Eigen::Vector3d minimum = values.front();
  Eigen::Vector3d maximum = values.front();
  for (std::size_t i = 0; i < values.size(); ++i) {
    const Eigen::Vector3d delta = values[i] - result.bias;
    const double norm = delta.norm();
    const double error = norm - result.measured_gravity;
    squared += error * error;
    jacobian.row(i) << -delta.transpose() / std::max(norm, 1e-12), -1.0;
    minimum = minimum.cwiseMin(values[i]);
    maximum = maximum.cwiseMax(values[i]);
  }
  result.quality.rms = std::sqrt(squared / values.size());
  const Eigen::Matrix4d information = jacobian.transpose() * jacobian;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eig(information);
  const double smallest = std::max(eig.eigenvalues().minCoeff(), 1e-16);
  result.quality.condition = eig.eigenvalues().maxCoeff() / smallest;
  const double variance = squared / std::max<double>(1.0, values.size() - 4.0);
  const Eigen::Matrix4d parameter_covariance =
      variance * information.completeOrthogonalDecomposition().pseudoInverse();
  result.covariance = parameter_covariance.topLeftCorner<3, 3>();
  result.gravity_stddev = std::sqrt(std::max(0.0, parameter_covariance(3, 3)));
  const double minimum_span = (maximum - minimum).minCoeff();
  result.quality.passed =
      result.quality.rms <= policy.accel_max_rms_m_s2 &&
      result.quality.condition <= policy.accel_max_condition &&
      minimum_span >= policy.accel_min_axis_span_m_s2;
  std::ostringstream message;
  message << (result.quality.passed ? "Passed" : "Failed")
          << ": sphere RMS " << result.quality.rms << " m/s^2, condition "
          << result.quality.condition << ", minimum axis span " << minimum_span
          << " m/s^2. Fitted magnitude is not asserted to be exact local gravity.";
  result.quality.summary = message.str();
  return result;
}

PlanarMagResult estimatePlanarMagBias(
    const std::vector<Sample>& magnetic,
    const std::vector<Sample>& accelerometer,
    const CalibrationPolicy& policy) {
  PlanarMagResult result;
  if (magnetic.size() < policy.minimum_samples ||
      accelerometer.size() != magnetic.size()) {
    result.quality.summary = "Magnetic and gravity samples must be paired";
    return result;
  }
  std::vector<Eigen::Vector3d> gravity;
  gravity.reserve(accelerometer.size());
  for (const auto& sample : accelerometer)
    gravity.push_back(sample.value.normalized());
  Eigen::Vector3d normal = robustCenter(gravity).normalized();
  if (!normal.allFinite()) {
    result.quality.summary = "Invalid gravity plane";
    return result;
  }
  result.plane_normal = normal;
  Eigen::Vector3d seed = std::abs(normal.z()) < 0.8 ? Eigen::Vector3d::UnitZ()
                                                    : Eigen::Vector3d::UnitX();
  const Eigen::Vector3d u = (seed - seed.dot(normal) * normal).normalized();
  const Eigen::Vector3d v = normal.cross(u).normalized();
  Eigen::MatrixXd design(magnetic.size(), 3);
  Eigen::VectorXd rhs(magnetic.size());
  for (std::size_t i = 0; i < magnetic.size(); ++i) {
    const double x = magnetic[i].value.dot(u);
    const double y = magnetic[i].value.dot(v);
    design.row(i) << 2.0 * x, 2.0 * y, 1.0;
    rhs(i) = x * x + y * y;
  }
  const Eigen::Vector3d fit = design.colPivHouseholderQr().solve(rhs);
  const double radius_squared = fit(2) + fit.head<2>().squaredNorm();
  if (!(radius_squared > 0.0)) {
    result.quality.summary = "Planar circle fit was nonphysical";
    return result;
  }
  result.radius = std::sqrt(radius_squared);
  result.in_plane_bias = fit(0) * u + fit(1) * v;
  std::vector<double> angles;
  double squared = 0.0;
  double tilt_squared = 0.0;
  for (std::size_t i = 0; i < magnetic.size(); ++i) {
    const Eigen::Vector3d centered = magnetic[i].value - result.in_plane_bias;
    const double x = centered.dot(u);
    const double y = centered.dot(v);
    const double error = std::hypot(x, y) - result.radius;
    squared += error * error;
    angles.push_back(std::atan2(y, x));
    const double angle = std::acos(std::clamp(gravity[i].dot(normal), -1.0, 1.0));
    tilt_squared += angle * angle;
  }
  std::sort(angles.begin(), angles.end());
  double max_gap = 0.0;
  for (std::size_t i = 1; i < angles.size(); ++i)
    max_gap = std::max(max_gap, angles[i] - angles[i - 1]);
  max_gap = std::max(max_gap, 2.0 * M_PI + angles.front() - angles.back());
  result.coverage_deg = (2.0 * M_PI - max_gap) * 180.0 / M_PI;
  result.quality.rms = std::sqrt(squared / magnetic.size());
  const double tilt_rms_deg = std::sqrt(tilt_squared / gravity.size()) * 180.0 / M_PI;
  result.quality.passed =
      result.quality.rms <= policy.mag_max_rms_fraction * result.radius &&
      result.coverage_deg >= policy.mag_min_coverage_deg &&
      tilt_rms_deg <= policy.mag_max_tilt_stddev_deg;
  std::ostringstream message;
  message << (result.quality.passed ? "Passed" : "Failed")
          << ": planar RMS " << result.quality.rms << ", heading coverage "
          << result.coverage_deg << " deg, gravity-plane variation "
          << tilt_rms_deg << " deg. Only in-plane hard iron is observable.";
  result.quality.summary = message.str();
  return result;
}

GravityReport makeGravityReport(const std::vector<Sample>& samples,
                                const CalibrationPolicy& policy) {
  GravityReport result;
  const auto values = vectors(samples);
  if (values.size() < policy.minimum_samples) {
    result.quality.summary = "Insufficient stationary accelerometer samples";
    return result;
  }
  result.specific_force_mean = robustCenter(values);
  result.gravity_vector = -result.specific_force_mean;
  result.covariance = covariance(values, result.specific_force_mean) /
                      static_cast<double>(values.size());
  result.magnitude = result.specific_force_mean.norm();
  if (result.magnitude <= 0.0) {
    result.quality.summary = "Invalid zero gravity magnitude";
    return result;
  }
  const Eigen::Vector3d direction = result.specific_force_mean / result.magnitude;
  const Eigen::Vector3d gravity_direction = -direction;
  result.magnitude_stddev = std::sqrt(std::max(
      0.0, (direction.transpose() * result.covariance * direction).value()));
  result.tilt_from_sensor_z_deg =
      std::acos(std::clamp(gravity_direction.dot(Eigen::Vector3d::UnitZ()), -1.0, 1.0)) *
      180.0 / M_PI;
  result.tilt_stddev_deg =
      std::sqrt(std::max(0.0, result.covariance.trace())) / result.magnitude *
      180.0 / M_PI;
  result.quality.rms = std::sqrt(covariance(values, result.specific_force_mean).trace());
  result.quality.passed = result.quality.rms <= policy.accel_max_rms_m_s2;
  result.quality.summary = result.quality.passed
      ? "Passed: sensor tilt relative to gravity only; yaw and mounting error are unobservable."
      : "Failed: motion/noise is too high for a gravity report.";
  return result;
}

Eigen::Vector3d composeCompensationBias(const Eigen::Matrix3d& compensation,
                                        const Eigen::Matrix3d& output_rotation,
                                        const Eigen::Vector3d& existing_bias,
                                        const Eigen::Vector3d& output_residual) {
  Eigen::FullPivLU<Eigen::Matrix3d> decomposition(compensation);
  Eigen::FullPivLU<Eigen::Matrix3d> rotation_decomposition(output_rotation);
  if (!decomposition.isInvertible() || !rotation_decomposition.isInvertible())
    throw std::invalid_argument("Compensation or reference rotation matrix is singular");
  return existing_bias + decomposition.solve(
      rotation_decomposition.solve(output_residual));
}

}  // namespace riptide_imu
