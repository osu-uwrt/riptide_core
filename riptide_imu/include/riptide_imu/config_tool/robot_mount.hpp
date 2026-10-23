#pragma once

#include <Eigen/Core>
#include <QString>
#include <optional>

namespace riptide_imu {

// Where the IMU sits on a robot, from the robot's description config (riptide_descriptions/config/talos.yaml,
// mercury_descriptions/.../mercury_xacro_frames.yaml): an `imu` entry whose `pose` is [x, y, z, roll, pitch, yaw]
// in meters and radians, as the xacro turns it into the imu_link joint. The imu_link axes are the sensor's own
// printed X / Y / Z, and base_link is ROS standard: X forward, Y left, Z up.
struct RobotMount {
  QString name;  // the file's stem, e.g. "talos"
  QString path;
  QString parent;  // what the pose was given relative to
  Eigen::Vector3d xyz{Eigen::Vector3d::Zero()};  // as written
  Eigen::Vector3d rpy{Eigen::Vector3d::Zero()};  // as written, radians
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};  // relative to base_link, meters
  Eigen::Matrix3d sensor_to_base{Eigen::Matrix3d::Identity()};  // columns: sensor axes in base_link
};

// Finds the IMU pose in a robot config. Accepts a top-level `imu`, or the first mapping with "imu" in its key
// anywhere in the file, holding a six-number `pose`. A top-level `base_link: [x, y, z]` means the pose is relative
// to the robot origin (as in talos.yaml), so the base_link offset is subtracted for `position`.
std::optional<RobotMount> loadRobotMount(const QString& path, QString* error);

// URDF / xacro rpy: fixed-axis roll about X, then pitch about Y, then yaw about Z (R = Rz * Ry * Rx).
Eigen::Matrix3d rpyToMatrix(const Eigen::Vector3d& rpy);
Eigen::Vector3d matrixToRpy(const Eigen::Matrix3d& rotation);

// "right (starboard)", "down" ... for a direction in base_link, or "between axes" when it is not within 10 degrees
// of one.
QString baseDirectionName(const Eigen::Vector3d& direction_in_base);

}  // namespace riptide_imu
