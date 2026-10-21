#include <QApplication>
#include <QIcon>
#include <QSettings>
#include <QSocketNotifier>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <csignal>
#include <cstdio>
#include <memory>
#include <unistd.h>

#include "riptide_imu/config_tool/main_window.hpp"
#include "riptide_imu/config_tool/theme.hpp"

namespace {

// SIGINT / SIGTERM reach Qt through a pipe, the only thing a handler can safely do. A second signal while the
// first is being handled exits at once, in case shutdown hangs.
int signal_pipe[2] = {-1, -1};
std::atomic<int> signals_received{0};

void onSignal(int signal_number) {
  if (signals_received.fetch_add(1) > 0) _exit(128 + signal_number);
  const char byte = static_cast<char>(signal_number);
  [[maybe_unused]] const auto written = write(signal_pipe[1], &byte, 1);
}

void installSignalHandlers() {
  if (pipe(signal_pipe) != 0) {
    std::perror("sensor_config: signal pipe");
    return;
  }
  struct sigaction action {};
  action.sa_handler = onSignal;
  sigemptyset(&action.sa_mask);
  action.sa_flags = SA_RESTART;
  sigaction(SIGINT, &action, nullptr);
  sigaction(SIGTERM, &action, nullptr);
}

}  // namespace

int main(int argc, char** argv) {
  // rclcpp's own handler only shuts down the ROS context and would leave the Qt event loop running.
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  installSignalHandlers();
  auto node = std::make_shared<rclcpp::Node>("vn_sensor_config");
  const auto connection_mode = node->declare_parameter<std::string>("connection_mode", "ros");
  const auto remote_namespace = node->declare_parameter<std::string>("remote_namespace", "");
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
  if (signal_pipe[0] >= 0) {
    auto* notifier = new QSocketNotifier(signal_pipe[0], QSocketNotifier::Read, &application);
    QObject::connect(notifier, &QSocketNotifier::activated, &application, [&application, notifier] {
      char byte = 0;
      [[maybe_unused]] const auto bytes = read(signal_pipe[0], &byte, 1);
      notifier->setEnabled(false);
      std::fprintf(stderr, "sensor_config: %s received, closing (again to force)\n",
                   byte == SIGTERM ? "SIGTERM" : "SIGINT");
      // Leaves every event loop, including an open dialog's.
      application.quit();
    });
  }
  const QString share_directory =
      QString::fromStdString(ament_index_cpp::get_package_share_directory("riptide_imu")) + "/config_tool_assets";
  riptide_imu::applyTheme(&application, QSettings().value("appearance/theme").toString(), share_directory);
  // Every size from config_tool_assets/icons/make_icons.py, so the dock and title bar each get a sharp one.
  QIcon icon;
  for (int size : {16, 24, 32, 48, 64, 128, 256, 512})
    icon.addFile(QString("%1/icons/sensor_config-%2.png").arg(share_directory).arg(size), QSize(size, size));
  application.setWindowIcon(icon);
  const QString initial_target = connection_mode == "ros" ?
      QString::fromStdString(remote_namespace) : QString();
  auto window = std::make_unique<riptide_imu::MainWindow>(
      initial_target, baud, QString::fromStdString(config_file), demo, share_directory,
      nullptr, nullptr, QString::fromStdString(connection_mode));
  window->show();
  const int result = application.exec();
  // Close the window (saving its layout, leaving maintenance, releasing the sensor) while ROS is still up.
  window.reset();
  node.reset();
  rclcpp::shutdown();
  return result;
}
