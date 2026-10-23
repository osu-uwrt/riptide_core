#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "riptide_imu/config_tool/calibration_guide.hpp"

namespace riptide_imu {
namespace {

constexpr double kGravity = 9.80;

// VN-100 convention: at rest the accelerometer reads specific force, -g along the gravity direction.
Eigen::Vector3d specificForce(const Eigen::Vector3d& gravity_in_body) {
  return -kGravity * gravity_in_body.normalized();
}

// Tilt `degrees` from +Z toward `azimuth_deg`, as the guide's poses are defined.
Eigen::Vector3d tilted(double degrees, double azimuth_deg) {
  return TiltTarget{"", degrees, azimuth_deg}.direction();
}

}  // namespace

TEST(CalibrationGuide, GravityDirectionIsOppositeTheSpecificForce) {
  EXPECT_TRUE(gravityDirection({0, 0, -9.8}).isApprox(Eigen::Vector3d::UnitZ()));
  // upside down, as a sensor mounted Z-up in a level robot sits
  EXPECT_TRUE(gravityDirection({0, 0, 9.8}).isApprox(-Eigen::Vector3d::UnitZ()));
  // +X end low: gravity leans toward +X, so the specific force leans toward -X.
  const Eigen::Vector3d nose_down = gravityDirection(specificForce(tilted(30, 0)));
  EXPECT_GT(nose_down.x(), 0.4);
  EXPECT_NEAR(angleBetweenDeg(nose_down, Eigen::Vector3d::UnitZ()), 30.0, 1e-9);
  EXPECT_TRUE(gravityDirection(Eigen::Vector3d::Zero()).isApprox(Eigen::Vector3d::UnitZ()));
}

TEST(CalibrationGuide, StillnessNeedsAFullQuietWindow) {
  StillnessMonitor monitor(1.0, 0.003, 0.04);
  EXPECT_FALSE(monitor.still());
  for (int i = 0; i < 5; ++i) monitor.add(i * 0.05, Eigen::Vector3d::Zero(), {0, 0, -9.8});
  EXPECT_FALSE(monitor.still()) << "a quarter-second window must not count as settled";
  for (int i = 5; i < 25; ++i) monitor.add(i * 0.05, {0.0001, 0, 0}, {0, 0, -9.8});
  EXPECT_TRUE(monitor.still());
  monitor.add(1.3, {0.05, 0, 0}, {0, 0, -9.8});
  EXPECT_FALSE(monitor.still()) << "one jolt in the window is movement";
  EXPECT_GT(monitor.gyroSpread(), 0.003);
}

TEST(CalibrationGuide, PosesAreFlatPlusEightEdgesAndCorners) {
  const auto poses = accelerometerPoses();
  ASSERT_EQ(poses.size(), 9u);
  EXPECT_TRUE(poses[0].direction().isApprox(Eigen::Vector3d::UnitZ()));
  for (std::size_t i = 1; i < poses.size(); ++i)
    EXPECT_NEAR(angleBetweenDeg(poses[i].direction(), Eigen::Vector3d::UnitZ()), 45.0, 1e-9);
  EXPECT_GT(poses[1].direction().x(), 0.5);  // "+X end down"
  EXPECT_GT(poses[3].direction().y(), 0.5);  // "+Y side down"
}

TEST(CalibrationGuide, ShallowTiltIsNotAcceptedForATiltedPose) {
  const auto poses = accelerometerPoses();
  EXPECT_TRUE(poseReached(poses[1], tilted(45, 0), 12));
  EXPECT_TRUE(poseReached(poses[1], tilted(41, 8), 12));
  EXPECT_FALSE(poseReached(poses[1], tilted(34, 0), 12)) << "inside the ring but too shallow";
  EXPECT_FALSE(poseReached(poses[1], tilted(45, 30), 12)) << "wrong direction";
  EXPECT_TRUE(poseReached(poses[0], tilted(8, 200), 12));
}

// The least-tipped accepted capture must still pass the accelerometer gate (hardware once gave a Z span of 1.94 m/s^2).
TEST(CalibrationGuide, ShallowestAcceptedPosesStillPassTheAccelerometerGate) {
  const CalibrationPolicy policy;
  const Eigen::Vector3d bias(0.04, -0.03, 0.025);
  std::mt19937 generator(7);
  std::normal_distribution<double> noise(0.0, 0.008);
  std::vector<Sample> samples;
  for (const auto& pose : accelerometerPoses()) {
    const double shallowest = pose.tilt_deg > 0 ? pose.tilt_deg - kPoseShortfallDeg + 0.01 : 0.0;
    const Eigen::Vector3d direction = tilted(shallowest, pose.azimuth_deg);
    ASSERT_TRUE(poseReached(pose, direction, 12));
    for (int i = 0; i < 50; ++i) {
      const Eigen::Vector3d reading =
          specificForce(direction) + bias + Eigen::Vector3d(noise(generator), noise(generator), noise(generator));
      samples.push_back({i * 0.05, reading, 25.0});
    }
  }
  const auto result = estimateAccelerometerBias(samples, policy);
  EXPECT_TRUE(result.quality.passed) << result.quality.summary;
  EXPECT_GE(result.axis_span.minCoeff(), policy.accel_min_axis_span_m_s2);
  // Within the estimator's own uncertainty: shallower poses separate bias from radius less well.
  const double sigma = std::sqrt(result.covariance.trace());
  EXPECT_LT((result.bias - bias).norm(), 3.0 * sigma) << "sigma " << sigma;
  EXPECT_LT((result.bias - bias).norm(), 0.02);
}

TEST(CalibrationGuide, HeadingCoverageIntegratesTurnAboutGravity) {
  HeadingCoverage coverage;
  const Eigen::Vector3d accel(0, 0, -9.8);
  coverage.reset(gravityDirection(accel));
  EXPECT_DOUBLE_EQ(coverage.coverageDeg(), 0.0);
  // 12 deg/s about the sensor Z (the gravity axis when level) for 20 s: 240 degrees.
  const double rate = 12.0 * M_PI / 180.0;
  for (int i = 0; i <= 400; ++i) coverage.add(i * 0.05, {0, 0, rate}, accel);
  EXPECT_NEAR(coverage.headingDeg(), 240.0, 0.5);
  EXPECT_NEAR(coverage.coverageDeg(), 240.0, 0.5);
  const auto turned = coverage.sectors();
  EXPECT_EQ(std::count(turned.begin(), turned.end(), true), 24);
  EXPECT_NEAR(coverage.yawRateDegS(), 12.0, 1e-6);
  EXPECT_LT(coverage.tiltVariationDeg(), 1e-6);
  for (int i = 401; i <= 620; ++i) coverage.add(i * 0.05, {0, 0, rate}, accel);
  EXPECT_DOUBLE_EQ(coverage.coverageDeg(), 360.0);
  coverage.add(31.05, Eigen::Vector3d::Zero(), specificForce(tilted(10, 90)));
  EXPECT_NEAR(coverage.tiltVariationDeg(), 10.0, 1e-6);
}

// The reported bug: 20 degrees "covered" before anything moved, from noise wrapping the heading past 0.
TEST(CalibrationGuide, HeadingCoverageStartsAtZeroAndIgnoresJitterAndBias) {
  HeadingCoverage coverage;
  const Eigen::Vector3d accel(0, 0, -9.8);
  const Eigen::Vector3d bias(0.0004, -0.0002, -0.0075);  // a raw gyro bias like the robot's
  coverage.reset(gravityDirection(accel), bias);
  for (int i = 0; i <= 600; ++i)
    coverage.add(i * 0.05, bias + Eigen::Vector3d(0, 0, 0.002 * std::sin(i * 1.3)), accel);
  EXPECT_LT(coverage.coverageDeg(), 1.0);
  const auto sectors = coverage.sectors();
  EXPECT_EQ(std::count(sectors.begin(), sectors.end(), true), 0);
  // Turning back and forth over the same 90 degrees covers 90, not more.
  const double rate = 15.0 * M_PI / 180.0;
  double t = 30.05;
  for (int sweep = 0; sweep < 4; ++sweep)
    for (int i = 0; i < 120; ++i, t += 0.05) coverage.add(t, bias + Eigen::Vector3d(0, 0, sweep % 2 ? -rate : rate),
                                                          accel);
  EXPECT_NEAR(coverage.coverageDeg(), 90.0, 1.0);
}

TEST(CalibrationGuide, RobotFramePosesSpeakInRobotDirections) {
  // talos.yaml's imu: roll 180, yaw -90 (sensor +X to starboard, +Y aft, +Z down)
  const Eigen::Matrix3d sensor_to_base =
      (Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
  const auto frame = robotGuideFrame("talos", sensor_to_base);
  const auto poses = accelerometerPoses(45.0, frame);
  ASSERT_EQ(poses.size(), 9u);
  EXPECT_EQ(poses[0].label, "Level");
  EXPECT_EQ(poses[1].label, "Front down");
  // Level robot: gravity along sensor +Z for this mount.
  EXPECT_TRUE(poses[0].direction().isApprox(Eigen::Vector3d::UnitZ(), 1e-9));
  // Front down: gravity leans toward the robot's front, which is the sensor's -Y.
  EXPECT_LT(poses[1].direction().y(), -0.6);
  // Right side down: toward starboard, the sensor's +X.
  EXPECT_GT(poses[3].direction().x(), 0.6);
  // A Z-up mount (mercury's identity rpy) is level with gravity along sensor -Z, and its tilts are still 45 deg.
  const auto upside = accelerometerPoses(45.0, robotGuideFrame("mercury", Eigen::Matrix3d::Identity()));
  EXPECT_TRUE(upside[0].direction().isApprox(-Eigen::Vector3d::UnitZ(), 1e-9));
  EXPECT_TRUE(poseReached(upside[2], upside[2].direction(), 12));
  EXPECT_NEAR(angleBetweenDeg(upside[5].direction(), upside[5].level()), 45.0, 1e-9);
}

TEST(CalibrationGuide, GravityGivesRollPitchAndAMountCorrection) {
  // Sensor rolled 3 degrees right side down and pitched 2 degrees nose up, VN-100 convention.
  const Eigen::Matrix3d attitude = (Eigen::AngleAxisd(2 * M_PI / 180, Eigen::Vector3d::UnitY()) *
                                    Eigen::AngleAxisd(3 * M_PI / 180, Eigen::Vector3d::UnitX()))
                                       .toRotationMatrix();
  const Eigen::Vector3d down = attitude.transpose() * Eigen::Vector3d::UnitZ();
  const Eigen::Vector2d roll_pitch = levelRollPitchDeg(down);
  EXPECT_NEAR(roll_pitch.x(), 3.0, 1e-9);
  EXPECT_NEAR(roll_pitch.y(), 2.0, 1e-9);
  const GravityReport report = makeGravityReport(
      std::vector<Sample>(300, Sample{0.0, specificForce(down), 25.0}), CalibrationPolicy{});
  EXPECT_NEAR(report.roll_deg, 3.0, 1e-6);
  EXPECT_NEAR(report.pitch_deg, 2.0, 1e-6);

  // A sensor meant to sit flat (FRD in a FLU base) but mounted with that tilt: the robot reads off level, and the
  // corrected rotation makes the same reading level.
  const Eigen::Matrix3d nominal = Eigen::Vector3d(1, -1, -1).asDiagonal();
  const auto check = checkMount(down, nominal);
  EXPECT_NEAR(check.error_deg, angleBetweenDeg(down, Eigen::Vector3d::UnitZ()), 1e-9);
  EXPECT_NEAR(check.roll_deg, 3.0, 1e-6);
  EXPECT_NEAR(check.pitch_deg, -2.0, 1e-6) << "ROS pitch is positive nose down";
  const Eigen::Matrix3d corrected =
      (Eigen::AngleAxisd(check.corrected_rpy.z(), Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(check.corrected_rpy.y(), Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(check.corrected_rpy.x(), Eigen::Vector3d::UnitX())).toRotationMatrix();
  EXPECT_TRUE((corrected * down).isApprox(-Eigen::Vector3d::UnitZ(), 1e-9));
}

TEST(CalibrationGuide, ProposalAndDeliveryFollowTheRegisters) {
  CalibrationSession session;
  const CalibrationPolicy policy;
  for (int i = 0; i < 300; ++i) session.gyro.push_back({i * 0.05, {0.001, -0.002, 0.0005}, 25.0});
  session.evaluate(CalibrationStep::Gyro, policy);
  QMap<int, QString> device{{84, "1;0;0;0;1;0;0;0;1;0.01;0;0"}, {26, "1;0;0;0;1;0;0;0;1"}};
  auto proposal = proposeCompensation(session, CalibrationStep::Gyro, device);
  EXPECT_EQ(proposal.unread, QList<int>{43});
  device[43] = "+00.000000;+00.000000;+00.000000";
  proposal = proposeCompensation(session, CalibrationStep::Gyro, device);
  ASSERT_TRUE(proposal.error.isEmpty()) << proposal.error.toStdString();
  EXPECT_TRUE(proposal.new_bias.isApprox(Eigen::Vector3d(0.011, -0.002, 0.0005), 1e-6));

  QMap<int, QString> staged;
  EXPECT_EQ(deliveryState(session, CalibrationStep::Gyro, device, staged), Delivery::None);
  session.delivered[0] = proposal.values;
  staged[84] = proposal.values;
  EXPECT_EQ(deliveryState(session, CalibrationStep::Gyro, device, staged), Delivery::Staged);
  staged.clear();  // discarded
  EXPECT_EQ(deliveryState(session, CalibrationStep::Gyro, device, staged), Delivery::None);
  // applied: the sensor echoes it at its own precision
  device[84] = "1;0;0;0;1;0;0;0;1;0.0110000;-0.00200000;0.000500000";
  EXPECT_EQ(deliveryState(session, CalibrationStep::Gyro, device, staged), Delivery::Applied);
}

TEST(CalibrationGuide, SessionEvaluatesAndReportsEveryCapture) {
  CalibrationSession session;
  const CalibrationPolicy policy;
  EXPECT_FALSE(session.evaluate(CalibrationStep::Gyro, policy));
  for (int i = 0; i < 300; ++i) {
    session.gyro.push_back({i * 0.05, {0.001, -0.002, 0.0005}, 25.0});
    session.gravity.push_back({i * 0.05, {0, 0, -kGravity}, 25.0});
  }
  const QJsonObject report = calibrationReport(session, policy);
  EXPECT_TRUE(report["gyro"].toObject()["evaluated"].toBool());
  EXPECT_TRUE(report["gyro"].toObject()["passed"].toBool());
  EXPECT_TRUE(report["gravity_report"].toObject()["passed"].toBool());
  EXPECT_FALSE(report["accelerometer"].toObject()["evaluated"].toBool());
  EXPECT_EQ(report["gyro"].toObject()["sample_count"].toInt(), 300);
  EXPECT_TRUE(session.gyro_result.residual_bias.isApprox(Eigen::Vector3d(0.001, -0.002, 0.0005)));
  session.clear(CalibrationStep::Gyro);
  EXPECT_EQ(session.sampleCount(CalibrationStep::Gyro), 0u);
  EXPECT_FALSE(session.evaluated[0]);
}

}  // namespace riptide_imu
