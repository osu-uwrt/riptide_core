#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <vector>

namespace riptide_imu {

struct Sample {
  double time_s{0.0};
  Eigen::Vector3d value{Eigen::Vector3d::Zero()};
  double temperature_c{0.0};
};

struct Quality {
  bool passed{false};
  std::string summary;
  double rms{0.0};
  double condition{0.0};
};

struct GyroResult {
  Eigen::Vector3d residual_bias{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d covariance{Eigen::Matrix3d::Zero()};
  double temperature_span_c{0.0};
  Quality quality;
};

struct AccelerometerResult {
  Eigen::Vector3d bias{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d covariance{Eigen::Matrix3d::Zero()};
  Eigen::Vector3d axis_span{Eigen::Vector3d::Zero()};
  double measured_gravity{0.0};
  double gravity_stddev{0.0};
  Quality quality;
};

struct PlanarMagResult {
  Eigen::Vector3d in_plane_bias{Eigen::Vector3d::Zero()};
  Eigen::Vector3d plane_normal{Eigen::Vector3d::UnitZ()};
  double radius{0.0};
  double coverage_deg{0.0};
  Quality quality;
};

struct GravityReport {
  Eigen::Vector3d specific_force_mean{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gravity_vector{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d covariance{Eigen::Matrix3d::Zero()};
  double magnitude{0.0};
  double magnitude_stddev{0.0};
  double tilt_from_sensor_z_deg{0.0};
  double tilt_stddev_deg{0.0};
  Quality quality;
};

struct CalibrationPolicy {
  std::size_t minimum_samples{200};
  std::size_t gyro_blocks{10};
  double gyro_max_rms_rad_s{0.003};
  double gyro_max_temperature_span_c{1.0};
  double gyro_max_pose_delta_rad_s{0.0015};
  double accel_max_rms_m_s2{0.08};
  double accel_max_condition{5000.0};
  double accel_min_axis_span_m_s2{2.0};
  double mag_max_rms_fraction{0.08};
  double mag_min_coverage_deg{300.0};
  double mag_max_tilt_stddev_deg{8.0};
};

CalibrationPolicy loadCalibrationPolicy(const std::string& path);

GyroResult estimateGyroBias(const std::vector<Sample>& samples,
                            const CalibrationPolicy& policy);
AccelerometerResult estimateAccelerometerBias(
    const std::vector<Sample>& samples, const CalibrationPolicy& policy);
PlanarMagResult estimatePlanarMagBias(
    const std::vector<Sample>& magnetic,
    const std::vector<Sample>& accelerometer,
    const CalibrationPolicy& policy);
GravityReport makeGravityReport(const std::vector<Sample>& samples,
                                const CalibrationPolicy& policy);

// Register compensation is y = C(x - B).  A residual measured in y-space is
// removed by B_new = B_old + C^-1 R^-1 residual_output, where R is the user
// reference-frame rotation. This is separate from the filter startup bias.
Eigen::Vector3d composeCompensationBias(const Eigen::Matrix3d& compensation,
                                        const Eigen::Matrix3d& output_rotation,
                                        const Eigen::Vector3d& existing_bias,
                                        const Eigen::Vector3d& output_residual);

}  // namespace riptide_imu
