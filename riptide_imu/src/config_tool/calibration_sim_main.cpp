#include <ament_index_cpp/get_package_share_directory.hpp>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <iostream>
#include <optional>

#include "riptide_imu/config_tool/calibration_simulation.hpp"

namespace {

QJsonArray vectorJson(const Eigen::Vector3d& value) {
  return {value.x(), value.y(), value.z()};
}

QJsonObject qualityJson(bool actual, bool expected, const riptide_imu::Quality& quality) {
  return {{"actual_pass", actual}, {"expected_pass", expected},
          {"matched", actual == expected}, {"summary", QString::fromStdString(quality.summary)}};
}

}  // namespace

int main(int argc, char** argv) {
  QString config = QString::fromStdString(
      ament_index_cpp::get_package_share_directory("riptide_imu")) +
      "/config/calibration_simulation.json";
  QString policy_path = QString::fromStdString(
      ament_index_cpp::get_package_share_directory("riptide_imu")) +
      "/config/calibration_policy.json";
  QString output;
  QString selected;
  for (int index = 1; index < argc; ++index) {
    const QString argument = argv[index];
    if (argument == "--config" && index + 1 < argc) config = argv[++index];
    else if (argument == "--policy" && index + 1 < argc) policy_path = argv[++index];
    else if (argument == "--output" && index + 1 < argc) output = argv[++index];
    else if (argument == "--scenario" && index + 1 < argc) selected = argv[++index];
    else if (argument == "--help") {
      std::cout << "Usage: vn_calibration_sim [--config FILE] [--policy FILE] "
                   "[--scenario NAME] [--output FILE]\n";
      return 0;
    } else {
      std::cerr << "Unknown or incomplete argument: " << argument.toStdString() << '\n';
      return 2;
    }
  }
  try {
    const auto suite = riptide_imu::loadCalibrationSimulationSuite(config.toStdString());
    const auto policy = riptide_imu::loadCalibrationPolicy(policy_path.toStdString());
    QJsonArray scenarios;
    bool all_matched = true;
    bool found = selected.isEmpty();
    for (const auto& scenario : suite.scenarios) {
      if (!selected.isEmpty() && selected != QString::fromStdString(scenario.name)) continue;
      found = true;
      const auto data = riptide_imu::generateCalibrationSimulation(suite, scenario);
      const auto result = riptide_imu::evaluateCalibrationSimulation(data, policy);
      const bool matched = riptide_imu::simulationMatchesExpected(result, scenario.expected);
      all_matched = all_matched && matched;
      QJsonObject record{
          {"name", QString::fromStdString(scenario.name)}, {"matched", matched},
          {"operator_lesson", QString::fromStdString(scenario.operator_lesson)},
          {"sample_counts", QJsonObject{{"gyro", static_cast<int>(data.gyro.size())},
              {"accelerometer", static_cast<int>(data.accelerometer.size())},
              {"magnetometer", static_cast<int>(data.magnetometer.size())},
              {"gravity", static_cast<int>(data.gravity.size())}}},
          {"gyro", qualityJson(result.gyro.quality.passed, scenario.expected.gyro,
                                result.gyro.quality)},
          {"accelerometer", qualityJson(result.accelerometer.quality.passed,
                                         scenario.expected.accelerometer,
                                         result.accelerometer.quality)},
          {"magnetometer", qualityJson(result.magnetometer.quality.passed,
                                        scenario.expected.magnetometer,
                                        result.magnetometer.quality)},
          {"gravity", qualityJson(result.gravity.quality.passed,
                                   scenario.expected.gravity,
                                   result.gravity.quality)},
          {"estimates", QJsonObject{
              {"gyro_bias_rad_s", vectorJson(result.gyro.residual_bias)},
              {"accelerometer_bias_m_s2", vectorJson(result.accelerometer.bias)},
              {"gravity_magnitude_m_s2", result.gravity.magnitude},
              {"gravity_tilt_deg", result.gravity.tilt_from_sensor_z_deg},
              {"magnetometer_coverage_deg", result.magnetometer.coverage_deg}}}};
      scenarios.push_back(record);
      std::cout << (matched ? "PASS " : "FAIL ") << scenario.name << '\n'
                << "  " << scenario.operator_lesson << '\n';
    }
    if (!found) {
      std::cerr << "Scenario not found: " << selected.toStdString() << '\n';
      return 2;
    }
    const QJsonObject report{{"schema_version", 1}, {"all_expectations_matched", all_matched},
                             {"config", config}, {"policy", policy_path},
                             {"scenarios", scenarios}};
    if (!output.isEmpty()) {
      QFile file(output);
      if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate))
        throw std::runtime_error(file.errorString().toStdString());
      file.write(QJsonDocument(report).toJson(QJsonDocument::Indented));
    }
    std::cout << "OVERALL: " << (all_matched ? "PASS" : "FAIL") << '\n';
    return all_matched ? 0 : 1;
  } catch (const std::exception& exception) {
    std::cerr << "Simulation failed: " << exception.what() << '\n';
    return 2;
  }
}
