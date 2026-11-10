// Copyright 2026 OSU Underwater Robotics Team
// SPDX-License-Identifier: BSD-3-Clause
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <riptide_simulink/simulink_model.hpp>
#include <robot_localization/srv/set_pose.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

namespace riptide_navigation
{
namespace
{

constexpr const char * kSensorNames[]{"imu", "fog", "dvl", "depth"};
constexpr char kImplementationVersion[] = "simulink_model_v1";

// The model publishes its numeric diagnostics in this order; see config/diagnostic_keys.json.
std::vector<std::string> read_keys(const std::string & path)
{
  std::ifstream file(path);
  if (!file) {throw std::runtime_error("Cannot read diagnostic keys " + path);}
  std::stringstream text;
  text << file.rdbuf();
  std::vector<std::string> keys;
  const std::string content = text.str();
  for (std::size_t start = content.find('"'); start != std::string::npos;
    start = content.find('"', start))
  {
    const std::size_t end = content.find('"', start + 1);
    keys.push_back(content.substr(start + 1, end - start - 1));
    start = end + 1;
  }
  return keys;
}

diagnostic_msgs::msg::KeyValue key_value(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

std::string set_pose_error(double code)
{
  switch (static_cast<int>(code)) {
    case 1: return "covariance is not finite and symmetric";
    case 2: return "covariance is not positive semidefinite";
    case 3: return "pose is not finite";
    case 4: return "quaternion norm is not 1";
    case 5: return "pose frame is not the world frame";
    default: return "";
  }
}

}  // namespace

// Supervises the generated talos_navigation model: loads its parameters and TF mounts, serves
// SetPose, and turns the model's numeric diagnostics into DiagnosticStatus.
// R2023a has no TF or service-server blocks, so those parts live here.
class TalosNavigationOverseer final : public rclcpp::Node
{
public:
  TalosNavigationOverseer()
  : Node("talos_navigation_overseer"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    model_name_ = declare_parameter<std::string>("model_node", "/talos/talos_navigation");
    model_file_ = declare_parameter<std::string>("model_parameter_file", "");
    keys_ = read_keys(declare_parameter<std::string>("diagnostic_keys_file", ""));
    world_frame_ = declare_parameter<std::string>("world_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_link_frame", "talos/base_link");
    mount_frames_ = {
      declare_parameter<std::string>("imu_frame_fallback", ""),
      declare_parameter<std::string>("fog_frame_fallback", ""),
      declare_parameter<std::string>("dvl_frame_fallback", "")};
    const double refresh = declare_parameter("mount_refresh_period", 1.0);
    clock_stall_timeout_ = declare_parameter("clock_stall_timeout", 2.0);
    heading_std_warning_ = declare_parameter("heading_std_warning", 30.0);
    position_offset_warning_ = declare_parameter("position_offset_warning", 0.05);
    max_stamp_lead_ = declare_parameter("max_stamp_lead", 1.0);
    publish_debug_ = declare_parameter("publish_debug_diagnostics", false);
    imu_acceleration_update_ =
      declare_parameter<std::string>("imu_acceleration_update", "consider");
    publication_mode_ = declare_parameter<std::string>("publication_mode", "queued_updates");
    const std::vector<std::string> configs{
      "vectornav_imu_config", "fog_twist_config", "dvl_twist_config", "depth_pose_config"};
    for (std::size_t i = 0; i < configs.size(); ++i) {
      const auto mask = declare_parameter<std::vector<bool>>(
        configs[i], std::vector<bool>(15, false));
      enabled_[i] = std::any_of(mask.begin(), mask.end(), [](bool value) {return value;});
      if (i == 0) {
        for (int group = 0; group < 3; ++group) {
          const int first = group == 0 ? 3 : group == 1 ? 9 : 12;
          imu_group_enabled_[group] = mask[first] || mask[first + 1] || mask[first + 2];
        }
      }
    }
    frame_fallbacks_ = {mount_frames_[0], mount_frames_[1], mount_frames_[2], ""};

    set_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("set_pose", 10);
    set_pose_service_ = create_service<robot_localization::srv::SetPose>(
      "set_pose", std::bind(
        &TalosNavigationOverseer::set_pose, this, std::placeholders::_1, std::placeholders::_2));
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    // depth can come in any world-fixed frame; learn it here for the model
    depth_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "depth/pose", rclcpp::SensorDataQoS(),
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        const auto & frame = msg->header.frame_id;
        if (!frame.empty() && frame != world_frame_ && frame != base_frame_) {
          std::lock_guard<std::mutex> lock(mutex_);
          depth_frame_ = frame;
        }
      });
    diagnostics_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "navigation/diagnostics", 10,
      std::bind(&TalosNavigationOverseer::diagnostics_callback, this, std::placeholders::_1));
    values_.assign(keys_.size(), 0.0);
    received_.assign(keys_.size(), false);
    last_report_wall_ = std::chrono::steady_clock::now();
    mount_timer_ = create_wall_timer(
      std::chrono::duration<double>(std::max(0.1, refresh)),
      std::bind(&TalosNavigationOverseer::refresh_mounts, this));
    watchdog_timer_ = create_wall_timer(
      std::chrono::seconds(1), std::bind(&TalosNavigationOverseer::watchdog, this));
  }

  // The wrapper needs a shared_ptr to this node, so it starts after construction.
  void start(const rclcpp::Node::SharedPtr & self)
  {
    riptide_simulink::ModelOptions options;
    options.reload_service = "reload_model_parameters";
    model_ = std::make_unique<riptide_simulink::SimulinkModel>(
      self, model_name_, options, [this]() {return model_parameters();});
    model_->on_reload(
      [this](const riptide_simulink::ModelResult & result) {
        if (!result.success) {
          RCLCPP_WARN(get_logger(), "Model parameter reload failed: %s", result.message.c_str());
        }
      });
  }

private:
  struct Mount
  {
    bool valid{};
    std::array<double, 3> translation{};
    std::array<double, 4> rotation{{1.0, 0.0, 0.0, 0.0}};
  };

  std::vector<rclcpp::Parameter> model_parameters()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto parameters = riptide_simulink::load_ros_parameters(model_file_, model_name_);
    const char * names[]{"imu", "fog", "dvl"};
    bool loaded = true;
    for (std::size_t i = 0; i < mounts_.size(); ++i) {
      const auto & mount = mounts_[i];
      const std::string prefix = std::string("navigation__mounts__") + names[i];
      parameters.emplace_back(prefix + "_mount_valid", mount.valid);
      // R2023a reads double[] parameters as int64[], so the model takes one double per element
      for (std::size_t k = 0; k < mount.translation.size(); ++k) {
        parameters.emplace_back(prefix + "_translation_" + std::to_string(k), mount.translation[k]);
      }
      for (std::size_t k = 0; k < mount.rotation.size(); ++k) {
        parameters.emplace_back(prefix + "_rotation_" + std::to_string(k), mount.rotation[k]);
      }
      loaded = loaded && (mount.valid || mount_frames_[i].empty());
    }
    parameters.emplace_back("navigation__mounts__depth_mount_valid", depth_mount_.valid);
    for (std::size_t k = 0; k < depth_mount_.translation.size(); ++k) {
      parameters.emplace_back(
        "navigation__mounts__depth_translation_" + std::to_string(k), depth_mount_.translation[k]);
    }
    for (std::size_t k = 0; k < depth_mount_.rotation.size(); ++k) {
      parameters.emplace_back(
        "navigation__mounts__depth_rotation_" + std::to_string(k), depth_mount_.rotation[k]);
    }
    // Get Parameter has no string type, so frame names go as zero-padded byte arrays
    const std::vector<std::pair<std::string, std::string>> frames{
      {"world_frame", world_frame_}, {"base_link_frame", base_frame_},
      {"imu_frame_fallback", mount_frames_[0]}, {"fog_frame_fallback", mount_frames_[1]},
      {"dvl_frame_fallback", mount_frames_[2]}, {"depth_frame", depth_mount_frame_}};
    for (const auto & [name, frame] : frames) {
      std::vector<uint8_t> bytes(64, 0);
      std::copy_n(frame.begin(), std::min<std::size_t>(frame.size(), bytes.size()), bytes.begin());
      const char * group = name == "depth_frame" ? "navigation__mounts__" : "navigation__frames__";
      parameters.emplace_back(group + name, bytes);
    }
    parameters.emplace_back("navigation__mounts__parameters_loaded", loaded);
    mounts_loaded_ = loaded;
    pushed_mounts_ = mounts_;
    pushed_depth_mount_ = depth_mount_;
    pushed_depth_frame_ = depth_mount_frame_;
    return parameters;
  }

  // Talos sensor mounts are rigid, so the latest transform is the mount.
  void refresh_mounts()
  {
    bool changed = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (std::size_t i = 0; i < mounts_.size(); ++i) {
        if (mount_frames_[i].empty()) {continue;}
        Mount mount;
        if (mount_frames_[i] == base_frame_) {
          mount.valid = true;
        } else {
          try {
            const auto transform =
              tf_buffer_.lookupTransform(base_frame_, mount_frames_[i], tf2::TimePointZero);
            const auto & t = transform.transform.translation;
            const auto & r = transform.transform.rotation;
            const double norm = std::sqrt(r.w * r.w + r.x * r.x + r.y * r.y + r.z * r.z);
            mount.valid = std::isfinite(norm) && norm > 1e-12;
            mount.translation = {t.x, t.y, t.z};
            mount.rotation = {r.w / norm, r.x / norm, r.y / norm, r.z / norm};
          } catch (const tf2::TransformException & error) {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 5000, "Cannot look up %s mount: %s",
              mount_frames_[i].c_str(), error.what());
          }
        }
        const auto & old = pushed_mounts_[i];
        changed = changed || mount.valid != old.valid || mount.translation != old.translation ||
          mount.rotation != old.rotation;
        mounts_[i] = mount;
      }
      if (!depth_frame_.empty()) {
        Mount mount;
        try {
          const auto transform =
            tf_buffer_.lookupTransform(world_frame_, depth_frame_, tf2::TimePointZero);
          const auto & t = transform.transform.translation;
          const auto & r = transform.transform.rotation;
          const double norm = std::sqrt(r.w * r.w + r.x * r.x + r.y * r.y + r.z * r.z);
          mount.valid = std::isfinite(norm) && norm > 1e-12;
          mount.translation = {t.x, t.y, t.z};
          mount.rotation = {r.w / norm, r.x / norm, r.y / norm, r.z / norm};
        } catch (const tf2::TransformException & error) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000, "Cannot look up depth frame %s: %s",
            depth_frame_.c_str(), error.what());
        }
        changed = changed || depth_frame_ != pushed_depth_frame_ ||
          mount.valid != pushed_depth_mount_.valid ||
          mount.translation != pushed_depth_mount_.translation ||
          mount.rotation != pushed_depth_mount_.rotation;
        depth_mount_ = mount;
        depth_mount_frame_ = depth_frame_;
      }
    }
    if (model_ && (changed || !mounts_loaded_) && model_->status().available) {
      model_->reload_parameters();
    }
  }

  // Forward a validated-in-model reset, after moving it into the world frame here.
  void set_pose(
    const std::shared_ptr<robot_localization::srv::SetPose::Request> request,
    std::shared_ptr<robot_localization::srv::SetPose::Response>)
  {
    auto pose = request->pose;
    if (!pose.header.frame_id.empty() && pose.header.frame_id != world_frame_) {
      try {
        geometry_msgs::msg::PoseWithCovarianceStamped transformed;
        tf2::doTransform(
          pose, transformed, tf_buffer_.lookupTransform(
            world_frame_, pose.header.frame_id, rclcpp::Time(pose.header.stamp, RCL_ROS_TIME)));
        pose = transformed;
      } catch (const tf2::TransformException & error) {
        // the model rejects the other frame and counts it
        RCLCPP_WARN(
          get_logger(), "Ignoring set_pose request: cannot transform from %s: %s",
          pose.header.frame_id.c_str(), error.what());
      }
    }
    set_pose_pub_->publish(pose);
  }

  void diagnostics_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const std::size_t offset = msg->layout.data_offset;
    for (std::size_t i = 0; i < msg->data.size() && offset + i < values_.size(); ++i) {
      values_[offset + i] = msg->data[i];
      received_[offset + i] = true;
    }
    if (std::all_of(received_.begin(), received_.end(), [](bool value) {return value;})) {
      received_.assign(received_.size(), false);
      clock_stalled_ = false;
      publish_diagnostics();
    }
  }

  // The model's clock stops with /clock, so keep reporting from wall time.
  void watchdog()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (std::chrono::steady_clock::now() - last_report_wall_ <
      std::chrono::duration<double>(clock_stall_timeout_))
    {
      return;
    }
    clock_stalled_ = true;
    publish_diagnostics();
  }

  double value(const std::string & key) const
  {
    const auto found = std::find(keys_.begin(), keys_.end(), key);
    return found == keys_.end() ? 0.0 : values_[found - keys_.begin()];
  }

  double sensor_value(std::size_t sensor, const std::string & key) const
  {
    return value(std::string(kSensorNames[sensor]) + "_" + key);
  }

  // Level and message from what changed since the last report, so an operator sees why data
  // is being dropped or distrusted right now.
  void report_faults(diagnostic_msgs::msg::DiagnosticStatus & status, double heading_std_deg)
  {
    std::vector<std::string> errors, warnings;
    auto changed = [this](std::size_t sensor, const std::string & key) {
        const std::string name = std::string(kSensorNames[sensor]) + "_" + key;
        return value(name) > baseline_[name];
      };
    // A clock mismatch delays every sensor; one late sensor is just latency
    bool any_arriving = false, all_lag = true;
    double min_lag_ms = 0.0;
    for (std::size_t i = 0; i < 4; ++i) {
      if (!changed(i, "arrivals") || sensor_value(i, "arrival_valid") == 0.0) {continue;}
      const double age = sensor_value(i, "arrival_age_ms");
      min_lag_ms = any_arriving ? std::min(min_lag_ms, age) : age;
      any_arriving = true;
      all_lag = all_lag && age > max_stamp_lead_ * 1000.0;
    }
    if (any_arriving && all_lag) {
      errors.push_back(
        "sensor stamps lag the node clock by " + std::to_string(min_lag_ms / 1000.0) +
        " s; check use_sim_time");
    }
    for (std::size_t i = 0; i < 4; ++i) {
      const std::string name = kSensorNames[i];
      const bool arriving = changed(i, "arrivals");
      const bool fused = changed(i, "accepted") || changed(i, "late_fallbacks");
      if (arriving && !fused && changed(i, "future_rejections")) {
        errors.push_back(
          name + " stamps lead the node clock by " +
          std::to_string(-sensor_value(i, "arrival_age_ms") / 1000.0) + " s; check use_sim_time");
      } else if (arriving && !fused && changed(i, "stale_rejections")) {
        errors.push_back(
          name + " arriving but stale (" + std::to_string(sensor_value(i, "late_lag_ms")) +
          " ms late)");
      }
      if (changed(i, "empty_frame_rejections")) {
        errors.push_back(name + " rejected: empty frame_id");
      }
      if (changed(i, "empty_frame_fallbacks")) {
        warnings.push_back(name + " empty frame_id, assumed " + frame_fallbacks_[i]);
      }
      if (changed(i, "queue_rejections")) {warnings.push_back(name + " queue overflow");}
      if (changed(i, "invalid_rejections")) {
        warnings.push_back(name + " invalid samples (stamp or values)");
      }
      if (changed(i, "innovation_rejections")) {
        const std::string key = name + "_innovation_rejections";
        warnings.push_back(
          name + " outliers rejected (" +
          std::to_string(static_cast<std::uint64_t>(value(key) - baseline_[key])) + ")");
      }
      if (changed(i, "late_fallbacks")) {
        warnings.push_back(
          name + " fused late (" + std::to_string(sensor_value(i, "late_lag_ms")) +
          " ms > history)");
      }
      if (changed(i, "frozen_rejections")) {warnings.push_back(name + " frozen");}
      if (changed(i, "garbage_rejected_windows")) {
        warnings.push_back(name + " rejecting inconsistent data");
      }
      if (changed(i, "gate_recoveries")) {warnings.push_back(name + " gate recovered to new data");}
      if (changed(i, "pre_clock_drops")) {warnings.push_back("waiting for /clock");}
    }
    const bool overloaded = value("overload_cycles") > baseline_["overload_cycles"];
    if (overloaded) {
      // Two reports in a row is sustained
      (overloaded_last_period_ ? errors : warnings).push_back("processing overload");
    }
    overloaded_last_period_ = overloaded;
    if (value("imu_invalid_quaternions") > baseline_["imu_invalid_quaternions"]) {
      warnings.push_back("imu quaternion not unit");
    }
    if (value("set_pose_rejections") > baseline_["set_pose_rejections"]) {
      warnings.push_back("set_pose rejected: " + set_pose_error(value("set_pose_error_code")));
    }
    if (value("fog_fallback_active") != 0.0) {warnings.push_back("FOG lost; VN yaw-rate fallback");}
    const double offset = value("published_position_offset_m");
    if (offset > position_offset_warning_) {
      warnings.push_back(
        "publishing a position correction gradually (" + std::to_string(offset) + " m left)");
    }
    if (heading_std_deg > heading_std_warning_) {
      warnings.push_back("heading unobserved (std " + std::to_string(heading_std_deg) + " deg)");
    }
    if (clock_stalled_) {warnings.push_back("node clock not advancing");}
    if (errors.empty() && warnings.empty()) {return;}
    // Errors first, then any existing timeout/group message, then warnings
    std::vector<std::string> parts = errors;
    if (status.level != diagnostic_msgs::msg::DiagnosticStatus::OK) {
      parts.push_back(status.message);
    }
    parts.insert(parts.end(), warnings.begin(), warnings.end());
    std::string message;
    for (const auto & part : parts) {message += (message.empty() ? "" : "; ") + part;}
    status.message = message;
    status.level = !errors.empty() ? diagnostic_msgs::msg::DiagnosticStatus::ERROR :
      std::max(status.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  }

  void publish_diagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = get_namespace() + std::string("/talos_navigation: estimator");
    status.hardware_id = "talos";
    const auto model = model_ ? model_->status() : riptide_simulink::ModelStatus{};
    const bool valid = value("state_valid") != 0.0;
    status.level = valid ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = valid ? "Quaternion EKF active" : "Invalid state or covariance";
    std::string missing;
    for (std::size_t i = 0; i < 4; ++i) {
      const double age = sensor_value(i, "measurement_age_ms");
      if (enabled_[i] && (age < 0.0 || sensor_value(i, "timeout_ms") < age)) {
        missing += (missing.empty() ? "" : ", ") + std::string(kSensorNames[i]);
      }
    }
    if (!model.available || !model.ready) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = model.available ? "Model parameters not loaded" : "Model node not running";
    } else if (valid && value("initialized") == 0.0) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "Waiting for an accepted measurement or pose reset";
    } else if (valid && !missing.empty()) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "Sensor timeout: " + missing;
    }
    const char * groups[]{"orientation", "angular_velocity", "acceleration"};
    for (int group = 0; group < 3; ++group) {
      if (valid && imu_group_enabled_[group] &&
        value(std::string("imu_") + groups[group] + "_available") == 0.0)
      {
        if (status.level == diagnostic_msgs::msg::DiagnosticStatus::OK) {
          status.message = "IMU group unavailable:";
        }
        status.level = std::max(status.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
        status.message += std::string(" ") + groups[group];
      }
    }
    const double heading_std_deg = value("heading_std_deg");
    if (valid) {report_faults(status, heading_std_deg);}
    for (std::size_t i = 0; i < keys_.size(); ++i) {baseline_[keys_[i]] = values_[i];}
    last_report_wall_ = std::chrono::steady_clock::now();

    const bool debug = publish_debug_;
    for (std::size_t i = 0; i < keys_.size(); ++i) {
      const auto & key = keys_[i];
      const bool debug_key = key == "queue_age_ms" || key == "filter_lag_ms" ||
        key.find("_innovation_norm") != std::string::npos ||
        key.find("_correction_norm") != std::string::npos;
      if (debug_key && !debug) {continue;}
      std::ostringstream text;
      text << values_[i];
      status.values.push_back(key_value(key, text.str()));
    }
    status.values.push_back(
      key_value("last_set_pose_error", set_pose_error(value("set_pose_error_code"))));
    status.values.push_back(key_value("implementation_version", kImplementationVersion));
    status.values.push_back(key_value("imu_acceleration_update", imu_acceleration_update_));
    status.values.push_back(key_value("publication_mode", publication_mode_));
    status.values.push_back(key_value("model_available", model.available ? "1" : "0"));
    status.values.push_back(key_value("model_ready", model.ready ? "1" : "0"));
    status.values.push_back(key_value("model_node", model.node_name));
    array.status.push_back(std::move(status));
    diagnostics_pub_->publish(array);
  }

  std::string model_name_, model_file_, world_frame_, base_frame_;
  std::string imu_acceleration_update_, publication_mode_;
  std::array<std::string, 3> mount_frames_;
  std::array<std::string, 4> frame_fallbacks_;
  std::array<bool, 4> enabled_{};
  std::array<bool, 3> imu_group_enabled_{};
  double clock_stall_timeout_{}, heading_std_warning_{}, position_offset_warning_{};
  double max_stamp_lead_{};
  bool publish_debug_{}, clock_stalled_{}, overloaded_last_period_{}, mounts_loaded_{};
  std::array<Mount, 3> mounts_{}, pushed_mounts_{};
  // depth_frame_ is the latest seen; depth_mount_frame_ is the one depth_mount_ resolves
  std::string depth_frame_, depth_mount_frame_, pushed_depth_frame_;
  Mount depth_mount_{}, pushed_depth_mount_{};
  std::vector<std::string> keys_;
  std::vector<double> values_;
  std::vector<bool> received_;
  std::map<std::string, double> baseline_;
  std::chrono::steady_clock::time_point last_report_wall_;
  std::mutex mutex_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<riptide_simulink::SimulinkModel> model_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr set_pose_pub_;
  rclcpp::Service<robot_localization::srv::SetPose>::SharedPtr set_pose_service_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr depth_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr diagnostics_sub_;
  rclcpp::TimerBase::SharedPtr mount_timer_, watchdog_timer_;
};

}  // namespace riptide_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<riptide_navigation::TalosNavigationOverseer>();
  node->start(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
