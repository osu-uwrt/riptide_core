#include <gtest/gtest.h>

#include <QFile>
#include <QTemporaryDir>
#include <cmath>

#include "riptide_imu/config_tool/drift_monitor.hpp"
#include "riptide_imu/config_tool/output_bandwidth.hpp"
#include "riptide_imu/config_tool/robot_mount.hpp"

namespace riptide_imu {

// The robot's sensor as read on 2026-10-16: binary output 1 on port 1 at 400 Hz plus VNYMR at 50 Hz.
const QMap<int, QString> kTalosOutput{{5, "460800"}, {6, "0"}, {7, "50"}, {75, "1;2;03;0530;0001"},
                                     {76, "0;0;00"}, {77, "0;0;00"}};

TEST(OutputBandwidth, MatchesTheRobotsBinaryOutput) {
  const auto config = parseBinaryOutput(kTalosOutput[75]);
  ASSERT_TRUE(config);
  EXPECT_EQ(config->async_mode, 1);
  EXPECT_DOUBLE_EQ(config->rateHz(), 400.0);
  // 4 header + common (2 + 16 + 12 + 12 + 20) + time (2 + 8)
  EXPECT_EQ(config->packetBytes(), 76);
  EXPECT_TRUE(config->unknownFields().isEmpty());
  EXPECT_EQ(formatBinaryOutput(*config), "1;2;03;0530;0001");

  const auto estimate = estimateBandwidth(kTalosOutput);
  ASSERT_TRUE(estimate.known);
  EXPECT_DOUBLE_EQ(estimate.bits_per_second, 76 * 10 * 400.0);
  EXPECT_NEAR(estimate.fraction(), 0.66, 0.005);
  EXPECT_FALSE(estimate.exceeds());
  EXPECT_DOUBLE_EQ(estimate.output_rate_hz, 400.0);
}

TEST(OutputBandwidth, FlagsAConfigurationTheBaudCannotCarry) {
  QMap<int, QString> registers = kTalosOutput;
  registers[75] = "1;1;13;0530;0001;0008";  // 800 Hz, plus the DCM from the attitude group
  registers[6] = "14";                      // and VNYMR ASCII at 50 Hz
  const auto estimate = estimateBandwidth(registers);
  EXPECT_TRUE(estimate.exceeds());
  EXPECT_EQ(estimate.parts.size(), 2);
  registers.remove(77);
  EXPECT_FALSE(estimateBandwidth(registers).known);
  EXPECT_EQ(estimateBandwidth(registers).missing, "77");
  // Port 2 output is noted, not counted against port 1.
  registers = kTalosOutput;
  registers[76] = "2;1;01;0010";
  EXPECT_DOUBLE_EQ(estimateBandwidth(registers).bits_per_second, 76 * 10 * 400.0);
  EXPECT_EQ(estimateBandwidth(registers).notes.size(), 1);
}

TEST(OutputBandwidth, RejectsMalformedBinaryOutput) {
  QString error;
  EXPECT_FALSE(parseBinaryOutput("1;2;03;0530", &error));
  EXPECT_FALSE(error.isEmpty());
  EXPECT_FALSE(parseBinaryOutput("1;2;01;0530;0001", &error));
  EXPECT_FALSE(parseBinaryOutput("4;2;00", &error));
  const auto off = parseBinaryOutput("0;0;00");
  ASSERT_TRUE(off);
  EXPECT_FALSE(off->hasOutput());
  EXPECT_DOUBLE_EQ(off->rateHz(), 0.0);
}

TEST(DriftMonitor, MeasuresSteadyYawDriftAndRestartsOnMotion) {
  DriftMonitor drift(60.0, 0.05, 10.0);
  // 0.5 deg/min of yaw drift through the +-180 wrap, sensor still.
  double t = 0.0;
  for (; t < 30.0; t += 0.05)
    drift.add(t, {1.0, -2.0, std::remainder(179.9 + 0.5 * t / 60.0 + 0.002 * std::sin(t * 7), 360.0)},
              {0.001, 0, 0.0005});
  const auto rate = drift.rateDegPerMinute();
  ASSERT_TRUE(rate);
  EXPECT_NEAR(rate->z(), 0.5, 0.02);
  EXPECT_NEAR(rate->x(), 0.0, 1e-9);
  ASSERT_TRUE(drift.sinceMark());
  EXPECT_NEAR(drift.sinceMark()->z(), 0.25, 0.02);
  // Picking it up restarts the rate, and the total says it moved.
  drift.add(t, {1.0, -2.0, 180.0}, {0, 0, 0.8});
  EXPECT_TRUE(drift.moving());
  EXPECT_FALSE(drift.rateDegPerMinute());
  EXPECT_TRUE(drift.movedSinceMark());
  drift.mark();
  EXPECT_FALSE(drift.movedSinceMark());
}

TEST(RobotMount, ReadsTalosAndMercuryConfigs) {
  QTemporaryDir directory;
  ASSERT_TRUE(directory.isValid());
  auto write = [&](const QString& name, const QByteArray& text) {
    QFile file(directory.filePath(name));
    EXPECT_TRUE(file.open(QIODevice::WriteOnly));
    file.write(text);
    return file.fileName();
  };
  // As in riptide_descriptions/config/talos.yaml: pose relative to the origin, base_link an offset from it.
  const QString talos = write("talos.yaml",
                              "base_link: [-0.14, 0.03, -0.09]\n"
                              "mass: 31.998\n"
                              "imu:\n"
                              "  rate: 100 #Hz\n"
                              "  pose: [-0.25, -0.1492, 0.0222, 3.141, 0.0, -1.5707]\n"
                              "depth:\n"
                              "  pose: [ -0.305, -0.125, 0.045, 0.0, 0.0, 0.0]\n");
  QString error;
  const auto mount = loadRobotMount(talos, &error);
  ASSERT_TRUE(mount) << error.toStdString();
  EXPECT_EQ(mount->name, "talos");
  EXPECT_TRUE(mount->position.isApprox(Eigen::Vector3d(-0.11, -0.1792, 0.1122), 1e-9));
  EXPECT_EQ(baseDirectionName(mount->sensor_to_base.col(0)), "right (starboard)");
  EXPECT_EQ(baseDirectionName(mount->sensor_to_base.col(1)), "back");
  EXPECT_EQ(baseDirectionName(mount->sensor_to_base.col(2)), "down");
  EXPECT_TRUE(matrixToRpy(rpyToMatrix(mount->rpy)).isApprox(mount->rpy, 1e-9));

  // As in mercury_xacro_frames.yaml: pose already relative to base_link.
  const auto mercury = loadRobotMount(write("mercury_xacro_frames.yaml",
                                            "robot_center: [0.18415, 0.06350, 0.02540]\n"
                                            "imu:\n"
                                            "  pose: [-0.07187, -0.0658, 0.02197, 0.0, 0.0, 0.0]\n"),
                                      &error);
  ASSERT_TRUE(mercury) << error.toStdString();
  EXPECT_TRUE(mercury->position.isApprox(Eigen::Vector3d(-0.07187, -0.0658, 0.02197), 1e-9));
  EXPECT_TRUE(mercury->sensor_to_base.isIdentity(1e-12));

  // Nested, differently named entries are found too; files without one say so.
  EXPECT_TRUE(loadRobotMount(write("nested.yaml", "sensors:\n  vn100_imu:\n    pose: [0, 0, 0, 0, 0, 1.5708]\n"),
                             &error));
  EXPECT_FALSE(loadRobotMount(write("none.yaml", "imu:\n  rate: 100\n"), &error));
  EXPECT_TRUE(error.contains("no imu entry"));
  EXPECT_FALSE(loadRobotMount(write("broken.yaml", "imu: [\n"), &error));
  EXPECT_FALSE(loadRobotMount(directory.filePath("missing.yaml"), &error));
}

}  // namespace riptide_imu
