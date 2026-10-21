#pragma once

#include <Eigen/Geometry>
#include <cstdint>
#include <string>
#include <vector>

#include "riptide_imu/config_tool/calibration.hpp"

namespace riptide_imu {

struct SimulationExpectedQuality {
  bool gyro{true};
  bool accelerometer{true};
  bool magnetometer{true};
  bool gravity{true};
};

struct CalibrationSimulationScenario {
  std::string name;
  std::string operator_lesson;
  std::uint32_t seed_offset{0};
  double gyro_motion_rad_s{0.0};
  double gyro_temperature_span_c{0.3};
  double accelerometer_pose_scale{1.0};
  double accelerometer_motion_m_s2{0.0};
  double magnetometer_coverage_deg{360.0};
  double magnetometer_tilt_variation_deg{0.0};
  double magnetometer_disturbance_gauss{0.0};
  double gravity_motion_m_s2{0.0};
  SimulationExpectedQuality expected;
};

struct CalibrationSimulationSuite {
  std::uint32_t seed{73129};
  double sample_rate_hz{20.0};
  double gravity_m_s2{9.79};
  double earth_rate_rad_s{7.2921159e-5};
  double latitude_deg{40.0};
  double gyro_noise_rad_s{0.00015};
  double accelerometer_noise_m_s2{0.008};
  double magnetometer_noise_gauss{0.0015};
  double temperature_c{25.0};
  Eigen::Vector3d gyro_bias_rad_s{0.0012, -0.0008, 0.00045};
  Eigen::Vector3d accelerometer_bias_m_s2{0.04, -0.03, 0.025};
  Eigen::Vector3d magnetic_bias_gauss{0.07, -0.045, 0.09};
  Eigen::Vector3d magnetic_field_world_gauss{0.22, 0.03, 0.41};
  std::vector<Eigen::Vector3d> stationary_poses_deg;
  std::vector<CalibrationSimulationScenario> scenarios;
};

struct CalibrationSimulationData {
  std::vector<Sample> gyro;
  std::vector<Sample> accelerometer;
  std::vector<Sample> magnetometer;
  std::vector<Sample> magnetometer_acceleration;
  std::vector<Sample> gravity;
};

struct CalibrationSimulationResult {
  GyroResult gyro;
  AccelerometerResult accelerometer;
  PlanarMagResult magnetometer;
  GravityReport gravity;
};

CalibrationSimulationSuite loadCalibrationSimulationSuite(
    const std::string& path);
CalibrationSimulationData generateCalibrationSimulation(
    const CalibrationSimulationSuite& suite,
    const CalibrationSimulationScenario& scenario);
CalibrationSimulationResult evaluateCalibrationSimulation(
    const CalibrationSimulationData& data,
    const CalibrationPolicy& policy);
bool simulationMatchesExpected(const CalibrationSimulationResult& result,
                               const SimulationExpectedQuality& expected);

}  // namespace riptide_imu
