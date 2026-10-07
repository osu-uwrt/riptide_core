#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <random>

#include "riptide_imu/config_tool/calibration.hpp"

using riptide_imu::Sample;

TEST(GyroCalibration, RecoversBiasWithCorrelatedNoiseAndOutliers) {
  riptide_imu::CalibrationPolicy policy;
  policy.gyro_max_pose_delta_rad_s = 0.002;
  const Eigen::Vector3d bias(0.0012, -0.0008, 0.00045);
  std::mt19937 generator(42);
  std::normal_distribution<double> noise(0.0, 0.00015);
  std::vector<Sample> samples;
  for (int i = 0; i < 1000; ++i) {
    Eigen::Vector3d value = bias + Eigen::Vector3d::Constant(noise(generator));
    value += Eigen::Vector3d(noise(generator), noise(generator), noise(generator));
    if (i == 217 || i == 733) value += Eigen::Vector3d(0.02, -0.01, 0.015);
    samples.push_back({i * 0.01, value, 24.0 + i * 0.0002});
  }
  const auto result = riptide_imu::estimateGyroBias(samples, policy);
  EXPECT_TRUE(result.quality.passed) << result.quality.summary;
  EXPECT_LT((result.residual_bias - bias).norm(), 8e-5);
  EXPECT_GT(result.covariance.trace(), 0.0);
}

TEST(GyroCalibration, RejectsMotionAndTemperatureDrift) {
  riptide_imu::CalibrationPolicy policy;
  std::vector<Sample> samples;
  for (int i = 0; i < 400; ++i)
    samples.push_back({i * 0.01, {0.02 * std::sin(i * 0.1), 0.0, 0.0}, 20.0 + i * 0.01});
  EXPECT_FALSE(riptide_imu::estimateGyroBias(samples, policy).quality.passed);
}

TEST(Compensation, AccountsForExistingMatrixAndReferenceRotation) {
  const Eigen::Matrix3d c = (Eigen::Matrix3d() << 1.01, .02, 0, -.01, .99, .01, 0, -.02, 1.02).finished();
  const Eigen::Matrix3d r = Eigen::AngleAxisd(0.43, Eigen::Vector3d(1,2,3).normalized()).toRotationMatrix();
  const Eigen::Vector3d old_bias(.1, -.2, .05);
  const Eigen::Vector3d raw_stationary(.12, -.18, .047);
  const Eigen::Vector3d residual = r * c * (raw_stationary - old_bias);
  const Eigen::Vector3d updated = riptide_imu::composeCompensationBias(c, r, old_bias, residual);
  EXPECT_LT((r * c * (raw_stationary - updated)).norm(), 1e-12);
  const Eigen::Vector3d second = riptide_imu::composeCompensationBias(c, r, updated, Eigen::Vector3d::Zero());
  EXPECT_LT((second - updated).norm(), 1e-14);
}

TEST(Compensation, RejectsSingularTransforms) {
  EXPECT_THROW(riptide_imu::composeCompensationBias(Eigen::Matrix3d::Zero(),
      Eigen::Matrix3d::Identity(), {}, {}), std::invalid_argument);
}

TEST(AccelerometerCalibration, RecoversBiasAndMagnitudeForArbitraryPoses) {
  riptide_imu::CalibrationPolicy policy;
  policy.accel_min_axis_span_m_s2 = 1.5;
  const Eigen::Vector3d bias(.12, -.08, .05);
  constexpr double gravity = 9.77;
  std::mt19937 generator(5);
  std::normal_distribution<double> noise(0.0, .008);
  std::vector<Sample> samples;
  for (int pose = 0; pose < 20; ++pose) {
    const double roll = (-42.0 + pose * 84.0 / 19.0) * M_PI / 180.0;
    const double pitch = (-38.0 + ((pose * 7) % 20) * 76.0 / 19.0) * M_PI / 180.0;
    const Eigen::Vector3d direction =
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::Vector3d::UnitZ();
    for (int i = 0; i < 20; ++i)
      samples.push_back({samples.size()*.02, bias + gravity*direction +
          Eigen::Vector3d(noise(generator),noise(generator),noise(generator)), 25.0});
  }
  const auto result = riptide_imu::estimateAccelerometerBias(samples, policy);
  EXPECT_TRUE(result.quality.passed) << result.quality.summary;
  EXPECT_LT((result.bias-bias).norm(), .02);
  EXPECT_NEAR(result.measured_gravity, gravity, .02);
}

TEST(AccelerometerCalibration, RejectsMisleadingSinglePoseSphere) {
  riptide_imu::CalibrationPolicy policy;
  std::vector<Sample> samples(400);
  for (int i=0;i<400;++i) samples[i] = {i*.01, {.1, -.1, 9.8 + 0.001*(i%3)}, 24.0};
  EXPECT_FALSE(riptide_imu::estimateAccelerometerBias(samples, policy).quality.passed);
}

TEST(MagnetometerCalibration, FitsCircleInGravityDerivedPlane) {
  riptide_imu::CalibrationPolicy policy;
  const Eigen::Vector3d normal = Eigen::Vector3d(.2,-.3,.93).normalized();
  const Eigen::Vector3d u = normal.unitOrthogonal();
  const Eigen::Vector3d v = normal.cross(u);
  const Eigen::Vector3d bias = .08*u - .04*v + .12*normal;
  std::vector<Sample> magnetic, acceleration;
  for(int i=0;i<400;++i){double a=2*M_PI*i/399.; magnetic.push_back({i*.02,bias+.42*(std::cos(a)*u+std::sin(a)*v),24}); acceleration.push_back({i*.02,9.78*normal,24});}
  const auto result = riptide_imu::estimatePlanarMagBias(magnetic, acceleration, policy);
  EXPECT_TRUE(result.quality.passed) << result.quality.summary;
  EXPECT_LT((result.in_plane_bias-(bias-bias.dot(normal)*normal)).norm(), 1e-5);
  EXPECT_NEAR(result.radius,.42,1e-5);
}

TEST(GravityReport, ReportsTiltWithoutClaimingMountingRotation) {
  riptide_imu::CalibrationPolicy policy;
  const Eigen::Vector3d force = Eigen::AngleAxisd(.3,Eigen::Vector3d::UnitX())*Eigen::Vector3d(0,0,9.75);
  std::vector<Sample> samples(300); for(int i=0;i<300;++i)samples[i]={i*.01,force,23};
  const auto report=riptide_imu::makeGravityReport(samples,policy);
  EXPECT_TRUE(report.quality.passed); EXPECT_NEAR(report.magnitude,9.75,1e-10);
  EXPECT_NEAR(report.tilt_from_sensor_z_deg,.3*180/M_PI,1e-10);
  EXPECT_LT((report.gravity_vector+force).norm(),1e-12);
}
