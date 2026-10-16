#include "riptide_imu/config_tool/calibration_simulation.hpp"

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>

namespace riptide_imu {
namespace {

Eigen::Vector3d vector3(const QJsonValue& value, const char* name) {
  const QJsonArray array = value.toArray();
  if (array.size() != 3) throw std::runtime_error(std::string(name) + " must contain three numbers");
  return {array[0].toDouble(), array[1].toDouble(), array[2].toDouble()};
}

Eigen::Matrix3d rotation(const Eigen::Vector3d& degrees) {
  const Eigen::Vector3d radians = degrees * M_PI / 180.0;
  return (Eigen::AngleAxisd(radians.z(), Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(radians.y(), Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(radians.x(), Eigen::Vector3d::UnitX())).toRotationMatrix();
}

Eigen::Vector3d gaussian(std::mt19937& generator, double sigma) {
  std::normal_distribution<double> distribution(0.0, sigma);
  return {distribution(generator), distribution(generator), distribution(generator)};
}

SimulationExpectedQuality expectedQuality(const QJsonObject& object) {
  return {object.value("gyro").toBool(true),
          object.value("accelerometer").toBool(true),
          object.value("magnetometer").toBool(true),
          object.value("gravity").toBool(true)};
}

}  // namespace

CalibrationSimulationSuite loadCalibrationSimulationSuite(const std::string& path) {
  QFile file(QString::fromStdString(path));
  if (!file.open(QIODevice::ReadOnly)) throw std::runtime_error(file.errorString().toStdString());
  QJsonParseError parse_error;
  const QJsonDocument document = QJsonDocument::fromJson(file.readAll(), &parse_error);
  if (parse_error.error != QJsonParseError::NoError || !document.isObject())
    throw std::runtime_error("Invalid calibration simulation JSON: " + parse_error.errorString().toStdString());
  const QJsonObject root = document.object();
  if (root.value("schema_version").toInt() != 1)
    throw std::runtime_error("Unsupported calibration simulation schema version");
  CalibrationSimulationSuite suite;
  suite.seed = static_cast<std::uint32_t>(root.value("seed").toInt(suite.seed));
  suite.sample_rate_hz = root.value("sample_rate_hz").toDouble(suite.sample_rate_hz);
  const QJsonObject truth = root.value("truth").toObject();
  suite.gravity_m_s2 = truth.value("gravity_m_s2").toDouble(suite.gravity_m_s2);
  suite.earth_rate_rad_s = truth.value("earth_rate_rad_s").toDouble(suite.earth_rate_rad_s);
  suite.latitude_deg = truth.value("latitude_deg").toDouble(suite.latitude_deg);
  suite.temperature_c = truth.value("temperature_c").toDouble(suite.temperature_c);
  suite.gyro_bias_rad_s = vector3(truth.value("gyro_bias_rad_s"), "gyro_bias_rad_s");
  suite.accelerometer_bias_m_s2 = vector3(
      truth.value("accelerometer_bias_m_s2"), "accelerometer_bias_m_s2");
  suite.magnetic_bias_gauss = vector3(truth.value("magnetic_bias_gauss"), "magnetic_bias_gauss");
  suite.magnetic_field_world_gauss = vector3(
      truth.value("magnetic_field_world_gauss"), "magnetic_field_world_gauss");
  const QJsonObject noise = root.value("noise").toObject();
  suite.gyro_noise_rad_s = noise.value("gyro_rad_s").toDouble(suite.gyro_noise_rad_s);
  suite.accelerometer_noise_m_s2 = noise.value("accelerometer_m_s2").toDouble(
      suite.accelerometer_noise_m_s2);
  suite.magnetometer_noise_gauss = noise.value("magnetometer_gauss").toDouble(
      suite.magnetometer_noise_gauss);
  for (const auto& pose : root.value("stationary_poses_deg").toArray())
    suite.stationary_poses_deg.push_back(vector3(pose, "stationary pose"));
  if (suite.stationary_poses_deg.size() < 4)
    throw std::runtime_error("At least four stationary poses are required");
  for (const auto& item : root.value("scenarios").toArray()) {
    const QJsonObject object = item.toObject();
    CalibrationSimulationScenario scenario;
    scenario.name = object.value("name").toString().toStdString();
    scenario.operator_lesson = object.value("operator_lesson").toString().toStdString();
    scenario.seed_offset = static_cast<std::uint32_t>(object.value("seed_offset").toInt());
    scenario.gyro_motion_rad_s = object.value("gyro_motion_rad_s").toDouble();
    scenario.gyro_temperature_span_c = object.value("gyro_temperature_span_c").toDouble(0.3);
    scenario.accelerometer_pose_scale = object.value("accelerometer_pose_scale").toDouble(1.0);
    scenario.accelerometer_motion_m_s2 = object.value("accelerometer_motion_m_s2").toDouble();
    scenario.magnetometer_coverage_deg = object.value("magnetometer_coverage_deg").toDouble(360.0);
    scenario.magnetometer_tilt_variation_deg = object.value("magnetometer_tilt_variation_deg").toDouble();
    scenario.magnetometer_disturbance_gauss = object.value("magnetometer_disturbance_gauss").toDouble();
    scenario.gravity_motion_m_s2 = object.value("gravity_motion_m_s2").toDouble();
    scenario.expected = expectedQuality(object.value("expected_pass").toObject());
    if (scenario.name.empty()) throw std::runtime_error("Simulation scenario name is required");
    suite.scenarios.push_back(scenario);
  }
  if (suite.scenarios.empty()) throw std::runtime_error("No calibration simulation scenarios configured");
  return suite;
}

CalibrationSimulationData generateCalibrationSimulation(
    const CalibrationSimulationSuite& suite,
    const CalibrationSimulationScenario& scenario) {
  CalibrationSimulationData data;
  std::mt19937 generator(suite.seed + scenario.seed_offset);
  const double dt = 1.0 / suite.sample_rate_hz;
  const Eigen::Vector3d gravity_world(0.0, 0.0, suite.gravity_m_s2);
  const double latitude = suite.latitude_deg * M_PI / 180.0;
  const Eigen::Vector3d earth_world = suite.earth_rate_rad_s *
      Eigen::Vector3d(std::cos(latitude), 0.0, std::sin(latitude));
  Eigen::Vector3d correlated_gyro_noise = Eigen::Vector3d::Zero();
  constexpr int samples_per_pose = 60;
  const int gyro_count = static_cast<int>(suite.stationary_poses_deg.size()) * samples_per_pose;
  for (int index = 0; index < gyro_count; ++index) {
    const auto& pose = suite.stationary_poses_deg[index / samples_per_pose];
    correlated_gyro_noise = 0.92 * correlated_gyro_noise +
        gaussian(generator, suite.gyro_noise_rad_s * std::sqrt(1.0 - 0.92 * 0.92));
    Eigen::Vector3d value = suite.gyro_bias_rad_s + rotation(pose).transpose() * earth_world +
        correlated_gyro_noise;
    value.x() += scenario.gyro_motion_rad_s * std::sin(index * 0.13);
    if ((index == 173 || index == 541) && scenario.gyro_motion_rad_s == 0.0)
      value += Eigen::Vector3d(0.008, -0.005, 0.006);
    const double fraction = static_cast<double>(index) / std::max(1, gyro_count - 1);
    data.gyro.push_back({index * dt, value,
        suite.temperature_c + scenario.gyro_temperature_span_c * fraction});
  }

  int accel_index = 0;
  for (const auto& configured_pose : suite.stationary_poses_deg) {
    Eigen::Vector3d pose = configured_pose;
    pose.x() *= scenario.accelerometer_pose_scale;
    pose.y() *= scenario.accelerometer_pose_scale;
    const Eigen::Vector3d specific_force = -rotation(pose).transpose() * gravity_world;
    for (int sample = 0; sample < samples_per_pose; ++sample, ++accel_index) {
      Eigen::Vector3d value = suite.accelerometer_bias_m_s2 + specific_force +
          gaussian(generator, suite.accelerometer_noise_m_s2);
      value += scenario.accelerometer_motion_m_s2 *
          Eigen::Vector3d(std::sin(accel_index * 0.19), std::cos(accel_index * 0.11),
                          std::sin(accel_index * 0.07));
      data.accelerometer.push_back({accel_index * dt, value, suite.temperature_c});
    }
  }

  constexpr int magnetometer_count = 420;
  for (int index = 0; index < magnetometer_count; ++index) {
    const double fraction = static_cast<double>(index) / (magnetometer_count - 1);
    const double yaw = -0.5 * scenario.magnetometer_coverage_deg +
        fraction * scenario.magnetometer_coverage_deg;
    const double varying_tilt = scenario.magnetometer_tilt_variation_deg *
        std::sin(2.0 * M_PI * fraction);
    const Eigen::Vector3d pose(12.0 + varying_tilt, -9.0, yaw);
    const Eigen::Matrix3d sensor_to_world = rotation(pose);
    Eigen::Vector3d magnetic = sensor_to_world.transpose() * suite.magnetic_field_world_gauss +
        suite.magnetic_bias_gauss + gaussian(generator, suite.magnetometer_noise_gauss);
    magnetic += scenario.magnetometer_disturbance_gauss *
        Eigen::Vector3d(std::sin(5.0 * yaw * M_PI / 180.0),
                        std::cos(3.0 * yaw * M_PI / 180.0), 0.2);
    const Eigen::Vector3d acceleration = -sensor_to_world.transpose() * gravity_world +
        suite.accelerometer_bias_m_s2 + gaussian(generator, suite.accelerometer_noise_m_s2);
    data.magnetometer.push_back({index * dt, magnetic, suite.temperature_c});
    data.magnetometer_acceleration.push_back({index * dt, acceleration, suite.temperature_c});
  }

  constexpr int gravity_count = 300;
  const Eigen::Matrix3d installed = rotation({2.5, -1.5, 17.0});
  for (int index = 0; index < gravity_count; ++index) {
    Eigen::Vector3d value = -installed.transpose() * gravity_world +
        suite.accelerometer_bias_m_s2 + gaussian(generator, suite.accelerometer_noise_m_s2);
    value += scenario.gravity_motion_m_s2 *
        Eigen::Vector3d(std::sin(index * 0.17), std::cos(index * 0.09),
                        std::sin(index * 0.23));
    data.gravity.push_back({index * dt, value, suite.temperature_c});
  }
  return data;
}

CalibrationSimulationResult evaluateCalibrationSimulation(
    const CalibrationSimulationData& data,
    const CalibrationPolicy& policy) {
  return {estimateGyroBias(data.gyro, policy),
          estimateAccelerometerBias(data.accelerometer, policy),
          estimatePlanarMagBias(data.magnetometer, data.magnetometer_acceleration, policy),
          makeGravityReport(data.gravity, policy)};
}

bool simulationMatchesExpected(const CalibrationSimulationResult& result,
                               const SimulationExpectedQuality& expected) {
  return result.gyro.quality.passed == expected.gyro &&
      result.accelerometer.quality.passed == expected.accelerometer &&
      result.magnetometer.quality.passed == expected.magnetometer &&
      result.gravity.quality.passed == expected.gravity;
}

}  // namespace riptide_imu
