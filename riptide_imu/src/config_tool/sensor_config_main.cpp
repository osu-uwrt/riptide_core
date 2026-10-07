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
  application.setStyleSheet(
      "QMainWindow,QWidget{font-size:10pt} QListWidget{background:#17212b;border:0;color:#dbe7f3;padding-top:18px}"
      "QListWidget::item{padding:14px 18px} QListWidget::item:selected{background:#2b77c0;color:white}"
      "QLabel#pageTitle{font-size:20pt;font-weight:600;margin:8px} QPushButton{padding:7px 12px}"
      "QWidget#connectionBar{background:palette(alternate-base);border:1px solid palette(mid);border-radius:6px}"
      "QPushButton#primaryButton{background:#2b77c0;color:white;border:0;border-radius:4px;font-weight:600}"
      "QTableWidget{gridline-color:#52606d} QStatusBar{padding:4px}");
  riptide_imu::MainWindow window(QString::fromStdString(imu_port), baud,
                                QString::fromStdString(config_file), demo,
                                QString::fromStdString(ament_index_cpp::get_package_share_directory("riptide_imu")));
  window.show();
  const int result = application.exec();
  rclcpp::shutdown();
  return result;
}
