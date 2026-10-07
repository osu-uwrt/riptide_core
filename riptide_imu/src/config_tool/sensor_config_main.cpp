#include <QApplication>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include "riptide_imu/config_tool/main_window.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("vn_sensor_config");
  const auto imu_port = node->declare_parameter<std::string>("imu_port", "/dev/ttyUSB0");
  const auto baud = node->declare_parameter<int>("baud", 460800);
  const auto config_file = node->declare_parameter<std::string>("config_file", "");
  const auto demo = node->declare_parameter<bool>("demo", false);

  std::vector<std::string> qt_arguments = rclcpp::remove_ros_arguments(argc, argv);
  std::vector<char*> qt_argv;
  for (auto& argument : qt_arguments) qt_argv.push_back(argument.data());
  int qt_argc = static_cast<int>(qt_argv.size());
  QApplication application(qt_argc, qt_argv.data());
  application.setApplicationName("VectorNav Workshop");
  application.setOrganizationName("OSU Underwater Robotics");
  riptide_imu::applyDarkTheme(&application);
  riptide_imu::MainWindow window(QString::fromStdString(imu_port), baud,
                                QString::fromStdString(config_file), demo,
                                QString::fromStdString(ament_index_cpp::get_package_share_directory("riptide_imu")));
  window.show();
  const int result = application.exec();
  rclcpp::shutdown();
  return result;
}
