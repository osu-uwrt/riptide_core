#include <gtest/gtest.h>
#include <algorithm>
#include <string>

#include "calibration_simulation.hpp"

TEST(CalibrationSimulation, EveryScenarioTriggersItsExpectedQualityGates) {
  const auto suite = riptide_imu::loadCalibrationSimulationSuite(
      std::string(TEST_DATA_DIRECTORY) + "/calibration_simulation.json");
  const auto policy = riptide_imu::loadCalibrationPolicy(
      std::string(TEST_SHARE_DIRECTORY) + "/config/calibration_policy.json");
  ASSERT_GE(suite.scenarios.size(), 9u);
  for (const auto& scenario : suite.scenarios) {
    SCOPED_TRACE(scenario.name);
    const auto data = riptide_imu::generateCalibrationSimulation(suite, scenario);
    const auto result = riptide_imu::evaluateCalibrationSimulation(data, policy);
    EXPECT_TRUE(riptide_imu::simulationMatchesExpected(result, scenario.expected))
        << "gyro: " << result.gyro.quality.summary << '\n'
        << "accelerometer: " << result.accelerometer.quality.summary << '\n'
        << "magnetometer: " << result.magnetometer.quality.summary << '\n'
        << "gravity: " << result.gravity.quality.summary;
    EXPECT_FALSE(scenario.operator_lesson.empty());
  }
}

TEST(CalibrationSimulation, NominalScenarioRecoversKnownBiases) {
  const auto suite = riptide_imu::loadCalibrationSimulationSuite(
      std::string(TEST_DATA_DIRECTORY) + "/calibration_simulation.json");
  const auto& scenario = suite.scenarios.front();
  ASSERT_EQ(scenario.name, "nominal");
  const auto data = riptide_imu::generateCalibrationSimulation(suite, scenario);
  const auto result = riptide_imu::evaluateCalibrationSimulation(
      data, riptide_imu::loadCalibrationPolicy(
          std::string(TEST_SHARE_DIRECTORY) + "/config/calibration_policy.json"));
  EXPECT_LT((result.gyro.residual_bias - suite.gyro_bias_rad_s).norm(), 2.0e-4);
  EXPECT_LT((result.accelerometer.bias - suite.accelerometer_bias_m_s2).norm(), 0.025);
  EXPECT_NEAR(result.accelerometer.measured_gravity, suite.gravity_m_s2, 0.02);
  EXPECT_GE(result.accelerometer.axis_span.minCoeff(), 2.0);
  EXPECT_NEAR(result.gravity.magnitude, suite.gravity_m_s2, 0.06);
}

TEST(CalibrationSimulation, LimitedTiltExposesWeakAxisCoverage) {
  const auto suite = riptide_imu::loadCalibrationSimulationSuite(
      std::string(TEST_DATA_DIRECTORY) + "/calibration_simulation.json");
  const auto iterator = std::find_if(
      suite.scenarios.begin(), suite.scenarios.end(), [](const auto& scenario) {
        return scenario.name == "accelerometer_limited_tilt";
      });
  ASSERT_NE(iterator, suite.scenarios.end());
  const auto result = riptide_imu::evaluateCalibrationSimulation(
      riptide_imu::generateCalibrationSimulation(suite, *iterator),
      riptide_imu::loadCalibrationPolicy(
          std::string(TEST_SHARE_DIRECTORY) + "/config/calibration_policy.json"));
  EXPECT_FALSE(result.accelerometer.quality.passed);
  EXPECT_LT(result.accelerometer.axis_span.minCoeff(), 2.0);
}

TEST(CalibrationSimulation, IsDeterministicForAConfiguredSeed) {
  const auto suite = riptide_imu::loadCalibrationSimulationSuite(
      std::string(TEST_DATA_DIRECTORY) + "/calibration_simulation.json");
  const auto first = riptide_imu::generateCalibrationSimulation(suite, suite.scenarios.front());
  const auto second = riptide_imu::generateCalibrationSimulation(suite, suite.scenarios.front());
  ASSERT_EQ(first.gyro.size(), second.gyro.size());
  ASSERT_EQ(first.accelerometer.size(), second.accelerometer.size());
  for (std::size_t index = 0; index < first.gyro.size(); ++index)
    EXPECT_EQ(first.gyro[index].value, second.gyro[index].value);
  for (std::size_t index = 0; index < first.accelerometer.size(); ++index)
    EXPECT_EQ(first.accelerometer[index].value, second.accelerometer[index].value);
}
