#include "riptide_imu/config_tool/robot_mount.hpp"

#include <Eigen/Geometry>
#include <QFileInfo>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <functional>

namespace riptide_imu {
namespace {

std::optional<std::vector<double>> numbers(const YAML::Node& node, std::size_t count) {
  if (!node || !node.IsSequence() || node.size() != count) return std::nullopt;
  std::vector<double> values;
  try {
    for (const auto& item : node) values.push_back(item.as<double>());
  } catch (const YAML::Exception&) {
    return std::nullopt;
  }
  return values;
}

std::string lower(std::string text) {
  std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) { return std::tolower(c); });
  return text;
}

}  // namespace

Eigen::Matrix3d rpyToMatrix(const Eigen::Vector3d& rpy) {
  return (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
}

Eigen::Vector3d matrixToRpy(const Eigen::Matrix3d& r) {
  return {std::atan2(r(2, 1), r(2, 2)), -std::asin(std::clamp(r(2, 0), -1.0, 1.0)), std::atan2(r(1, 0), r(0, 0))};
}

QString baseDirectionName(const Eigen::Vector3d& direction) {
  const Eigen::Vector3d d = direction.normalized();
  const std::pair<Eigen::Vector3d, const char*> axes[] = {
      {Eigen::Vector3d::UnitX(), "forward"}, {-Eigen::Vector3d::UnitX(), "back"},
      {Eigen::Vector3d::UnitY(), "left (port)"}, {-Eigen::Vector3d::UnitY(), "right (starboard)"},
      {Eigen::Vector3d::UnitZ(), "up"}, {-Eigen::Vector3d::UnitZ(), "down"}};
  for (const auto& [axis, name] : axes)
    if (d.dot(axis) > std::cos(10.0 * M_PI / 180.0)) return name;
  return "between axes";
}

std::optional<RobotMount> loadRobotMount(const QString& path, QString* error) {
  auto fail = [&](const QString& message) -> std::optional<RobotMount> {
    if (error) *error = message;
    return std::nullopt;
  };
  YAML::Node root;
  try {
    root = YAML::LoadFile(path.toStdString());
  } catch (const YAML::Exception& exception) {
    return fail(QString("Could not read %1: %2").arg(QFileInfo(path).fileName(), exception.what()));
  }
  if (!root.IsMap()) return fail(QFileInfo(path).fileName() + " is not a YAML mapping");

  // Top-level "imu" first, then any "*imu*" mapping with a pose, depth first in file order.
  std::optional<std::vector<double>> pose;
  std::string found_key;
  if (root["imu"] && root["imu"].IsMap()) {
    pose = numbers(root["imu"]["pose"], 6);
    if (pose) found_key = "imu";
  }
  std::function<void(const YAML::Node&, const std::string&)> search = [&](const YAML::Node& node,
                                                                           const std::string& prefix) {
    if (pose || !node.IsMap()) return;
    for (const auto& entry : node) {
      if (pose) return;
      std::string key;
      try {
        key = entry.first.as<std::string>();
      } catch (const YAML::Exception&) {
        continue;
      }
      const std::string name = prefix.empty() ? key : prefix + "." + key;
      if (lower(key).find("imu") != std::string::npos && entry.second.IsMap()) {
        pose = numbers(entry.second["pose"], 6);
        if (pose) {
          found_key = name;
          return;
        }
      }
      search(entry.second, name);
    }
  };
  search(root, "");
  if (!pose)
    return fail(QFileInfo(path).fileName() + " has no imu entry with pose: [x, y, z, roll, pitch, yaw]");

  RobotMount mount;
  mount.path = QFileInfo(path).absoluteFilePath();
  mount.name = QFileInfo(path).completeBaseName();
  mount.xyz = {(*pose)[0], (*pose)[1], (*pose)[2]};
  mount.rpy = {(*pose)[3], (*pose)[4], (*pose)[5]};
  mount.sensor_to_base = rpyToMatrix(mount.rpy);
  mount.position = mount.xyz;
  mount.parent = QString("base_link (%1.pose)").arg(QString::fromStdString(found_key));
  if (const auto base = numbers(root["base_link"], 3)) {
    mount.position -= Eigen::Vector3d((*base)[0], (*base)[1], (*base)[2]);
    mount.parent = QString("robot origin (%1.pose), less the base_link offset").arg(QString::fromStdString(found_key));
  }
  return mount;
}

}  // namespace riptide_imu
