// Copyright 2026 OSU Underwater Robotics Team
// SPDX-License-Identifier: BSD-3-Clause
#include "riptide_navigation/estimator_support.hpp"
#include "riptide_navigation/consider_imu.hpp"
#include "riptide_navigation/generated_core_access.hpp"
#include "riptide_navigation/pose_covariance.hpp"
#include "riptide_navigation/scalar_update.hpp"
#include "talos_ekf.h"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_localization/srv/set_pose.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace riptide_navigation
{
namespace
{

enum class Sensor : std::size_t {Imu = 0, Fog = 1, Dvl = 2, Depth = 3, Count = 4};

struct Measurement
{
  rclcpp::Time stamp;
  Sensor sensor;
  std::array<double, 16> value{};
  std::array<double, 256> covariance{};
  std::array<double, 3> offset{};
  std::array<bool, 3> imu_groups{{true, true, true}};
  std::uint64_t sequence{};
  std::int64_t arrival_stamp_ns{};
};

constexpr std::size_t kConfigSize = 15;
constexpr char kImplementationVersion[] = "raw_acceleration_tilt_v17";

std::array<double, kConfigSize> config_mask(const std::vector<bool> & values)
{
  std::array<double, kConfigSize> result{};
  std::transform(
    values.begin(), values.end(), result.begin(), [](bool value) {
      return value ? 1.0 : 0.0;
    });
  return result;
}

void validate_config(
  const std::string & name, const std::vector<bool> & values,
  const std::array<bool, kConfigSize> & supported)
{
  if (values.size() != kConfigSize) {
    throw std::invalid_argument(name + " must contain 15 booleans");
  }
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (values[i] && !supported[i]) {
      throw std::invalid_argument(
              name + " enables unsupported field at index " + std::to_string(i));
    }
  }
}

struct EarlierMeasurement
{
  bool operator()(const Measurement & a, const Measurement & b) const
  {
    return a.stamp == b.stamp ? a.sequence > b.sequence : a.stamp > b.stamp;
  }
};

double covariance_value(const std::array<double, 9> & cov, int row, int col, double fallback)
{
  if (cov[0] < 0.0) {return row == col ? fallback : 0.0;}
  const bool unspecified = std::all_of(
    cov.begin(), cov.end(), [](double value) {return value == 0.0;});
  if (unspecified) {return row == col ? fallback : 0.0;}
  const double value = cov[3 * row + col];
  if (!std::isfinite(value)) {return row == col ? fallback : 0.0;}
  return value;
}

bool covariance_unspecified(const std::array<double, 9> & covariance)
{
  return std::all_of(
    covariance.begin(), covariance.end(), [](double value) {return value == 0.0;});
}

void condition_covariance(
  std::array<double, 9> & covariance, double multiplier, double variance_floor)
{
  for (int row = 0; row < 3; ++row) {
    for (int col = row; col < 3; ++col) {
      const double value = 0.5 *
        (covariance[3 * row + col] + covariance[3 * col + row]) * multiplier;
      covariance[3 * row + col] = value;
      covariance[3 * col + row] = value;
    }
    covariance[3 * row + row] = std::max(covariance[3 * row + row], variance_floor);
  }
  for (int row = 0; row < 3; ++row) {
    double off_diagonal_sum = 0.0;
    for (int col = 0; col < 3; ++col) {
      if (row != col) {off_diagonal_sum += std::abs(covariance[3 * row + col]);}
    }
    covariance[3 * row + row] = std::max(
      covariance[3 * row + row], off_diagonal_sum + variance_floor);
  }
}

std::array<double, 9> rotate_covariance(const tf2::Matrix3x3 & r, const std::array<double, 9> & c)
{
  std::array<double, 9> out{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        for (int l = 0; l < 3; ++l) {
          out[3 * i + j] += r[i][k] * c[3 * k + l] * r[j][l];
        }
      }
    }
  }
  return out;
}

void set_block(
  std::array<double, 256> & dst, int offset, int size, const std::array<double,
  9> & src)
{
  for (int row = 0; row < size; ++row) {
    for (int col = 0; col < size; ++col) {
      dst[(offset + row) + 16 * (offset + col)] = src[3 * row + col];
    }
  }
}

void copy_masked_covariance(
  const Measurement & event, int size, const double * mask, double * destination)
{
  for (int row = 0; row < size; ++row) {
    for (int col = 0; col < size; ++col) {
      if (mask[row] != 0.0 && mask[col] != 0.0) {
        destination[row + size * col] = event.covariance[row + 16 * col];
      } else {
        destination[row + size * col] = row == col ? 1.0 : 0.0;
      }
    }
  }
}

diagnostic_msgs::msg::KeyValue key_value(const std::string & key, std::uint64_t value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = std::to_string(value);
  return result;
}

diagnostic_msgs::msg::KeyValue key_value(const std::string & key, double value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = std::to_string(value);
  return result;
}

diagnostic_msgs::msg::KeyValue key_value(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

}  // namespace

class TalosEkfNode final : public rclcpp::Node
{
public:
  TalosEkfNode()
  : Node("talos_ekf_node", rclcpp::NodeOptions().use_clock_thread(false)),
    tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    world_frame_ = declare_parameter("world_frame", "odom");
    base_frame_ = declare_parameter("base_link_frame", "talos/base_link");
    publish_tf_ = declare_parameter("publish_tf", true);
    frequency_ = declare_parameter("frequency", 30.0);
    reuse_preview_corrections_ = declare_parameter("reuse_preview_corrections", true);
    scalar_fog_correction_ = declare_parameter("scalar_fog_correction", true);
    initialize_tilt_from_imu_ = declare_parameter("initialize_tilt_from_imu", true);
    vectornav_tilt_correction_frequency_ =
      declare_parameter("vectornav_tilt_correction_frequency", 0.0);
    imu_orientation_covariance_coordinates_ =
      declare_parameter("imu_orientation_covariance_coordinates", "body_rotvec");
    if (imu_orientation_covariance_coordinates_ != "body_rotvec" &&
      imu_orientation_covariance_coordinates_ != "euler_rpy")
    {
      throw std::invalid_argument(
              "imu_orientation_covariance_coordinates must be body_rotvec or euler_rpy");
    }
    imu_angular_acceleration_time_constant_ =
      declare_parameter("imu_angular_acceleration_time_constant", 0.05);
    if (imu_angular_acceleration_time_constant_ <= 0.0) {
      throw std::invalid_argument("imu_angular_acceleration_time_constant must be positive");
    }
    gravity_ = declare_parameter("gravitational_acceleration", 9.755455);
    reorder_delay_ = declare_parameter("reorder_delay", 0.02);
    imu_acceleration_update_ = declare_parameter<std::string>("imu_acceleration_update", "joint");
    publication_mode_ = declare_parameter<std::string>("publication_mode", "predict");
    if (imu_acceleration_update_ != "joint" && imu_acceleration_update_ != "consider") {
      throw std::invalid_argument("imu_acceleration_update must be joint or consider");
    }
    if (publication_mode_ != "predict" && publication_mode_ != "queued_updates") {
      throw std::invalid_argument("publication_mode must be predict or queued_updates");
    }
    max_queue_size_ = static_cast<std::size_t>(declare_parameter("max_queue_size", 512));
    max_measurements_per_cycle_ = static_cast<std::size_t>(
      declare_parameter("max_measurements_per_cycle", 64));
    max_prediction_dt_ = declare_parameter("max_prediction_dt", 0.1);
    sensor_timeouts_ = declare_parameter<std::vector<double>>(
      "sensor_timeouts", {0.25, 0.25, 0.5, 0.5});
    if (sensor_timeouts_.size() != 4 || std::any_of(
        sensor_timeouts_.begin(), sensor_timeouts_.end(),
        [](double value) {return !std::isfinite(value) || value <= 0.0;}))
    {
      throw std::invalid_argument("sensor_timeouts must contain four finite positive seconds");
    }
    process_noise_diag_ = declare_parameter<std::vector<double>>(
      "process_noise_diag", {5e-5, 5e-5, 6e-5, 3e-5, 3e-5, 6e-5, 1e-6, 2.5e-5,
        2.5e-5, 4e-5, 3e-4, 3e-4, 6e-4, 1e-5, 1e-5, 1.5e-5});
    if (process_noise_diag_.size() != 16) {
      throw std::invalid_argument("process_noise_diag must contain 16 values");
    }
    reset_covariance_diag_ = declare_parameter<std::vector<double>>(
      "reset_covariance_diag", {1.0, 1.0, 0.25, 0.0, 2.5e-4, 2.5e-4, 2.5e-4,
        0.25, 0.25, 0.25, 0.05, 0.05, 0.05, 0.5, 0.5, 0.5});
    if (reset_covariance_diag_.size() != 16) {
      throw std::invalid_argument("reset_covariance_diag must contain 16 values");
    }
    default_orientation_variance_ = declare_parameter("default_orientation_variance", 2e-4);
    default_angular_velocity_variance_ =
      declare_parameter("default_angular_velocity_variance", 2e-5);
    default_acceleration_variance_ = declare_parameter("default_acceleration_variance", 2e-3);
    default_fog_variance_ = declare_parameter("default_fog_variance", 1e-6);
    default_dvl_variance_ = declare_parameter("default_dvl_variance", 2.5e-4);
    default_depth_variance_ = declare_parameter("default_depth_variance", 2.5e-3);
    imu_covariance_multiplier_ = declare_parameter("imu_covariance_multiplier", 1.0);
    imu_orientation_covariance_multiplier_ =
      declare_parameter("imu_orientation_covariance_multiplier", 1.0);
    imu_angular_velocity_covariance_multiplier_ =
      declare_parameter("imu_angular_velocity_covariance_multiplier", 1.0);
    imu_acceleration_covariance_multiplier_ =
      declare_parameter("imu_acceleration_covariance_multiplier", 1.0);
    for (const double multiplier : {imu_orientation_covariance_multiplier_,
        imu_angular_velocity_covariance_multiplier_, imu_acceleration_covariance_multiplier_})
    {
      if (!std::isfinite(multiplier) || multiplier <= 0.0) {
        throw std::invalid_argument(
                "IMU component covariance multipliers must be finite and positive");
      }
    }
    fog_covariance_multiplier_ = declare_parameter("fog_covariance_multiplier", 1.0);
    dvl_covariance_multiplier_ = declare_parameter("dvl_covariance_multiplier", 1.0);
    depth_covariance_multiplier_ = declare_parameter("depth_covariance_multiplier", 1.0);
    covariance_floor_ = declare_parameter("covariance_floor", 1e-9);
    dvl_rejection_threshold_ = declare_parameter("dvl_rejection_threshold", 5.0);
    depth_rejection_threshold_ = declare_parameter("depth_rejection_threshold", 5.0);
    if (!std::isfinite(dvl_rejection_threshold_) || dvl_rejection_threshold_ < 0.0 ||
      !std::isfinite(depth_rejection_threshold_) || depth_rejection_threshold_ < 0.0)
    {
      throw std::invalid_argument("Innovation rejection thresholds must be finite and nonnegative");
    }
    publish_debug_diagnostics_ = declare_parameter("publish_debug_diagnostics", false);

    const std::vector<bool> imu_default{
      false, false, false, true, true, false, false, false, false,
      true, true, false, true, true, true};
    const std::vector<bool> dvl_default{
      false, false, false, false, false, false, true, true, true,
      false, false, false, false, false, false};
    const std::vector<bool> fog_default{
      false, false, false, false, false, false, false, false, false,
      false, false, true, false, false, false};
    const std::vector<bool> depth_default{
      false, false, true, false, false, false, false, false, false,
      false, false, false, false, false, false};
    const std::array<bool, kConfigSize> imu_supported{
      false, false, false, true, true, false, false, false, false,
      true, true, true, true, true, true};
    const std::array<bool, kConfigSize> dvl_supported{
      false, false, false, false, false, false, true, true, true,
      false, false, false, false, false, false};
    const std::array<bool, kConfigSize> fog_supported{
      false, false, false, false, false, false, false, false, false,
      true, true, true, false, false, false};
    const std::array<bool, kConfigSize> depth_supported{
      false, false, true, false, false, false, false, false, false,
      false, false, false, false, false, false};
    const auto imu_config =
      declare_parameter<std::vector<bool>>("vectornav_imu_config", imu_default);
    const auto dvl_config =
      declare_parameter<std::vector<bool>>("dvl_twist_config", dvl_default);
    const auto fog_config =
      declare_parameter<std::vector<bool>>("fog_twist_config", fog_default);
    const auto depth_config =
      declare_parameter<std::vector<bool>>("depth_pose_config", depth_default);
    validate_config("vectornav_imu_config", imu_config, imu_supported);
    validate_config("dvl_twist_config", dvl_config, dvl_supported);
    validate_config("fog_twist_config", fog_config, fog_supported);
    validate_config("depth_pose_config", depth_config, depth_supported);
    imu_config_ = config_mask(imu_config);
    dvl_config_ = config_mask(dvl_config);
    fog_config_ = config_mask(fog_config);
    depth_config_ = config_mask(depth_config);

    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(100);
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      declare_parameter("imu_topic", "vectornav/imu"), sensor_qos,
      std::bind(&TalosEkfNode::imu_callback, this, std::placeholders::_1));
    fog_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      declare_parameter("fog_topic", "gyro/twist"), sensor_qos,
      std::bind(&TalosEkfNode::fog_callback, this, std::placeholders::_1));
    dvl_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      declare_parameter("dvl_topic", "dvl_twist"), sensor_qos,
      std::bind(&TalosEkfNode::dvl_callback, this, std::placeholders::_1));
    depth_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      declare_parameter("depth_topic", "depth/pose"), sensor_qos,
      std::bind(&TalosEkfNode::depth_callback, this, std::placeholders::_1));
    set_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "set_pose", 10, std::bind(&TalosEkfNode::set_pose_topic, this, std::placeholders::_1));
    reset_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    set_pose_service_ = create_service<robot_localization::srv::SetPose>(
      "set_pose",
      std::bind(
        &TalosEkfNode::set_pose_service, this, std::placeholders::_1,
        std::placeholders::_2), rmw_qos_profile_services_default, reset_callback_group_);

    odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      declare_parameter("odometry_topic", "odometry/filtered"), 10);
    acceleration_pub_ = create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>(
      declare_parameter("acceleration_topic", "accel/filtered"), 10);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, frequency_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&TalosEkfNode::timer_callback, this));
    State initial{};
    initial[3] = 1.0;
    reset_filter(initial, get_clock()->now());
    initialized_ = false;
    initial_tilt_pending_ = initialize_tilt_from_imu_;
    RCLCPP_INFO(
      get_logger(), "Estimator implementation %s; VectorNav tilt corrections limited to %.1f Hz",
      kImplementationVersion, vectornav_tilt_correction_frequency_);
  }

private:
  std::optional<geometry_msgs::msg::TransformStamped> sensor_transform(
    const std::string & frame, const rclcpp::Time & stamp, Sensor sensor)
  {
    if (frame.empty() || frame == base_frame_) {
      geometry_msgs::msg::TransformStamped identity;
      identity.header.stamp = stamp;
      identity.header.frame_id = base_frame_;
      identity.child_frame_id = frame;
      identity.transform.rotation.w = 1.0;
      return identity;
    }
    try {
      // Talos sensor mounts are rigid. Using the latest static mount avoids
      // rejecting samples when robot_state_publisher trails a sensor stamp.
      return tf_buffer_.lookupTransform(base_frame_, frame, tf2::TimePointZero);
    } catch (const tf2::TransformException & error) {
      ++transform_rejections_;
      ++transform_by_sensor_[static_cast<std::size_t>(sensor)];
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Cannot transform %s to %s: %s",
        frame.c_str(), base_frame_.c_str(), error.what());
      return std::nullopt;
    }
  }

  bool valid_stamp(const rclcpp::Time & stamp)
  {
    if (stamp.nanoseconds() <= 0) {
      ++invalid_rejections_;
      return false;
    }
    return true;
  }

  void enqueue(Measurement measurement)
  {
    std::lock_guard<std::mutex> lock(filter_mutex_);
    const std::size_t sensor = static_cast<std::size_t>(measurement.sensor);
    measurement.arrival_stamp_ns = get_clock()->now().nanoseconds();
    arrival_age_ms_[sensor] = (measurement.arrival_stamp_ns - measurement.stamp.nanoseconds()) *
      1e-6;
    if (arrival_counts_[sensor]++ == 0) {
      min_arrival_age_ms_[sensor] = max_arrival_age_ms_[sensor] = arrival_age_ms_[sensor];
    }
    min_arrival_age_ms_[sensor] = std::min(min_arrival_age_ms_[sensor], arrival_age_ms_[sensor]);
    max_arrival_age_ms_[sensor] = std::max(max_arrival_age_ms_[sensor], arrival_age_ms_[sensor]);
    if (!valid_stamp(measurement.stamp)) {++invalid_by_sensor_[sensor]; return;}
    const bool finite_measurement = std::all_of(
      measurement.value.begin(), measurement.value.end(),
      [](double value) {return std::isfinite(value);}) && std::all_of(
      measurement.covariance.begin(), measurement.covariance.end(),
      [](double value) {return std::isfinite(value);}) && std::all_of(
      measurement.offset.begin(), measurement.offset.end(),
      [](double value) {return std::isfinite(value);});
    if (!finite_measurement) {
      ++invalid_rejections_;
      ++invalid_by_sensor_[sensor];
      return;
    }
    if (queue_.size() >= max_queue_size_) {
      ++queue_rejections_;
      ++queue_by_sensor_[sensor];
      return;
    }
    measurement.sequence = sequence_++;
    queue_.push(std::move(measurement));
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
    auto transform = sensor_transform(msg->header.frame_id, stamp, Sensor::Imu);
    if (!transform) {return;}
    tf2::Quaternion q_bs;
    tf2::fromMsg(transform->transform.rotation, q_bs);
    q_bs.normalize();
    tf2::Quaternion q_ws(msg->orientation.x, msg->orientation.y, msg->orientation.z,
      msg->orientation.w);
    auto finite_vector = [](const auto & value) {
        return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
      };
    // ROS Imu marks each independently unavailable estimate with covariance[0]
    // == -1. Never treat that sentinel as an unknown-covariance fallback, or
    // discard valid rates/acceleration because orientation is unavailable.
    std::array<bool, 3> groups{{
      msg->orientation_covariance[0] != -1.0 &&
      q_ws.length2() >= 1e-12 && std::isfinite(q_ws.length2()),
      msg->angular_velocity_covariance[0] != -1.0 && finite_vector(msg->angular_velocity),
      msg->linear_acceleration_covariance[0] != -1.0 && finite_vector(msg->linear_acceleration)}};
    const auto & offset = transform->transform.translation;
    if (!groups[1] && offset.x * offset.x + offset.y * offset.y + offset.z * offset.z > 1e-24) {
      // A mounted accelerometer needs rate data to remove its rotational
      // acceleration. Defer that group instead of inventing a zero rate.
      groups[2] = false;
    }
    latest_imu_groups_ = groups;
    for (std::size_t group = 0; group < groups.size(); ++group) {
      if (!groups[group]) {++imu_unavailable_groups_[group];}
    }
    bool usable = false;
    for (int i = 0; i < 3; ++i) {
      usable = usable || (groups[0] && imu_config_[3 + i] != 0.0) ||
        (groups[1] && imu_config_[9 + i] != 0.0) ||
        (groups[2] && imu_config_[12 + i] != 0.0);
    }
    if (!usable) {return;}
    if (!groups[0]) {q_ws = tf2::Quaternion(0, 0, 0, 1);}
    q_ws.normalize();
    tf2::Quaternion q_wb = q_ws * q_bs.inverse();
    q_wb.normalize();
    const tf2::Vector3 gravity_body = tf2::quatRotate(q_wb.inverse(), tf2::Vector3(0, 0, 1));
    const tf2::Vector3 omega = groups[1] ? tf2::quatRotate(
      q_bs,
      tf2::Vector3(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z)) :
      tf2::Vector3(0, 0, 0);
    tf2::Vector3 accel = groups[2] ? tf2::quatRotate(
      q_bs,
      tf2::Vector3(
        msg->linear_acceleration.x, msg->linear_acceleration.y,
        msg->linear_acceleration.z)) : tf2::Vector3(0, 0, 0);
    // Fuse raw acceleration; gravity is predicted from the state in the core.

    Measurement event{stamp, Sensor::Imu};
    event.imu_groups = groups;
    event.offset = {transform->transform.translation.x,
      transform->transform.translation.y, transform->transform.translation.z};
    event.value[0] = gravity_body.x(); event.value[1] = gravity_body.y();
    event.value[2] = gravity_body.z();
    event.value[3] = omega.x(); event.value[4] = omega.y(); event.value[5] = omega.z();
    event.value[6] = accel.x(); event.value[7] = accel.y(); event.value[8] = accel.z();

    const tf2::Matrix3x3 r_bs(q_bs);
    std::array<double, 9> orientation{}, angular{}, acceleration{};
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        orientation[3 * row + col] = covariance_value(
          msg->orientation_covariance, row, col,
          default_orientation_variance_);
        angular[3 * row + col] = covariance_value(
          msg->angular_velocity_covariance, row, col,
          default_angular_velocity_variance_);
        acceleration[3 * row + col] = covariance_value(
          msg->linear_acceleration_covariance, row,
          col, default_acceleration_variance_);
      }
    }
    // Condition in the reported coordinates. Diagonal dominance after a
    // coordinate mapping would destroy the gravity/yaw covariance nullspace.
    condition_covariance(
      orientation,
      imu_covariance_multiplier_ * imu_orientation_covariance_multiplier_, covariance_floor_);
    condition_covariance(
      angular,
      imu_covariance_multiplier_ * imu_angular_velocity_covariance_multiplier_, covariance_floor_);
    condition_covariance(
      acceleration,
      imu_covariance_multiplier_ * imu_acceleration_covariance_multiplier_, covariance_floor_);
    if (imu_orientation_covariance_coordinates_ == "euler_rpy") {
      double roll, pitch, yaw;
      tf2::Matrix3x3(q_ws).getRPY(roll, pitch, yaw);
      // Map Euler perturbations to sensor-body rotation-vector perturbations.
      const tf2::Matrix3x3 e(1, 0, -std::sin(pitch), 0, std::cos(roll),
        std::sin(roll) * std::cos(pitch), 0, -std::sin(roll), std::cos(roll) * std::cos(pitch));
      orientation = rotate_covariance(e, orientation);
    }
    orientation = rotate_covariance(r_bs, orientation);
    angular = rotate_covariance(r_bs, angular);
    acceleration = rotate_covariance(r_bs, acceleration);
    // Gravity-direction covariance: J=-skew(g), so yaw about gravity is naturally removed.
    const double gx = gravity_body.x(), gy = gravity_body.y(), gz = gravity_body.z();
    const double j[9]{0, gz, -gy, -gz, 0, gx, gy, -gx, 0};
    std::array<double, 9> gravity_cov{};
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        for (int k = 0; k < 3; ++k) {
          for (int l = 0; l < 3; ++l) {
            gravity_cov[3 * row + col] += j[3 * row + k] * orientation[3 * k + l] * j[3 * col + l];
          }
        }
      }
    }
    for (int i = 0; i < 3; ++i) {gravity_cov[3 * i + i] += 1e-9;}
    std::array<double, 9> tilt_cov{};
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        for (int k = 0; k < 3; ++k) {
          for (int l = 0; l < 3; ++l) {
            tilt_cov[3 * row + col] +=
              j[3 * row + k] * gravity_cov[3 * k + l] * j[3 * col + l];
          }
        }
      }
    }
    for (int i = 0; i < 3; ++i) {tilt_cov[3 * i + i] += covariance_floor_;}
    set_block(event.covariance, 0, 3, tilt_cov);
    set_block(event.covariance, 3, 3, angular);
    // No artificial shared orientation noise: the acceleration payload has not
    // had the measured quaternion's gravity subtracted from it.
    set_block(event.covariance, 6, 3, acceleration);
    enqueue(std::move(event));
  }

  void fog_callback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
  {
    const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
    auto transform = sensor_transform(msg->header.frame_id, stamp, Sensor::Fog);
    if (!transform) {return;}
    tf2::Quaternion q;
    tf2::fromMsg(transform->transform.rotation, q);
    const tf2::Vector3 omega = tf2::quatRotate(
      q, tf2::Vector3(
        msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z));
    Measurement event{stamp, Sensor::Fog};
    event.value[0] = omega.x(); event.value[1] = omega.y(); event.value[2] = omega.z();
    std::array<double, 9> c{};
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        const double value = msg->twist.covariance[(row + 3) * 6 + col + 3];
        c[3 * row + col] = std::isfinite(value) ? value :
          (row == col ? default_fog_variance_ : 0.0);
      }
    }
    if (covariance_unspecified(c)) {
      for (int i = 0; i < 3; ++i) {c[3 * i + i] = default_fog_variance_;}
    }
    condition_covariance(c, fog_covariance_multiplier_, covariance_floor_);
    const auto rotated = rotate_covariance(tf2::Matrix3x3(q), c);
    set_block(event.covariance, 0, 3, rotated);
    enqueue(std::move(event));
  }

  void dvl_callback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
  {
    const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
    auto transform = sensor_transform(msg->header.frame_id, stamp, Sensor::Dvl);
    if (!transform) {return;}
    tf2::Quaternion q;
    tf2::fromMsg(transform->transform.rotation, q);
    const tf2::Vector3 velocity = tf2::quatRotate(
      q, tf2::Vector3(
        msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z));
    const tf2::Vector3 offset(transform->transform.translation.x,
      transform->transform.translation.y,
      transform->transform.translation.z);
    Measurement event{stamp, Sensor::Dvl};
    event.value[0] = velocity.x(); event.value[1] = velocity.y(); event.value[2] = velocity.z();
    event.offset = {offset.x(), offset.y(), offset.z()};
    std::array<double, 9> c{};
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        const double value = msg->twist.covariance[row * 6 + col];
        c[3 * row + col] = std::isfinite(value) ? value :
          (row == col ? default_dvl_variance_ : 0.0);
      }
    }
    if (covariance_unspecified(c)) {
      for (int i = 0; i < 3; ++i) {c[3 * i + i] = default_dvl_variance_;}
    }
    condition_covariance(c, dvl_covariance_multiplier_, covariance_floor_);
    auto rotated = rotate_covariance(tf2::Matrix3x3(q), c);
    condition_covariance(rotated, 1.0, covariance_floor_);
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        event.covariance[row + 16 * col] = rotated[3 * row + col];
      }
    }
    enqueue(std::move(event));
  }

  void depth_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
    double depth = msg->pose.pose.position.z;
    if (!msg->header.frame_id.empty() && msg->header.frame_id != world_frame_) {
      try {
        geometry_msgs::msg::PoseStamped source, transformed;
        source.header = msg->header;
        source.pose = msg->pose.pose;
        tf2::doTransform(
          source, transformed,
          tf_buffer_.lookupTransform(world_frame_, msg->header.frame_id, stamp));
        depth = transformed.pose.position.z;
      } catch (const tf2::TransformException &) {
        ++transform_rejections_;
        ++transform_by_sensor_[3];
        return;
      }
    }
    Measurement event{stamp, Sensor::Depth};
    event.value[0] = depth;
    const double variance = msg->pose.covariance[14];
    event.covariance[0] = std::max(
      (std::isfinite(variance) && variance > 0.0 ? variance : default_depth_variance_) *
      depth_covariance_multiplier_, covariance_floor_);
    enqueue(std::move(event));
  }

  void clear_inputs()
  {
    core_->rtU.enable_imu = core_->rtU.enable_fog = core_->rtU.enable_dvl = false;
    core_->rtU.enable_depth = core_->rtU.enable_reset = false;
  }

  void set_process_noise(double dt)
  {
    std::fill_n(core_->rtU.prediction_context + 1, 256, 0.0);
    core_->rtU.prediction_context[0] = dt;
    const double scale = std::max(dt, 0.0);
    for (int i = 0; i < 16; ++i) {
      if (i < 3 || i > 6) {
        core_->rtU.prediction_context[1 + i + 16 * i] = std::sqrt(process_noise_diag_[i] * scale);
      }
    }
    State normalized{};
    std::copy(std::begin(core_->rtDW.x), std::end(core_->rtDW.x), normalized.begin());
    normalize_quaternion(normalized);
    const double w = normalized[3], x = normalized[4];
    const double y = normalized[5], z = normalized[6];
    const double g[12] = {-x, -y, -z, w, -z, y, z, w, -x, -y, x, w};
    // Q=S_Q*S_Q'. Map the same body angular spectral densities directly
    // into the quaternion tangent; column 6 is the null radial direction.
    for (int axis = 0; axis < 3; ++axis) {
      const double root_density = 0.5 * std::sqrt(scale * process_noise_diag_[3 + axis]);
      for (int row = 0; row < 4; ++row) {
        core_->rtU.prediction_context[1 + (3 + row) + 16 * (3 + axis)] =
          root_density * g[row * 3 + axis];
      }
    }
  }

  void capture_output()
  {
    // Correction calls use dt=0,Q=0. Read the posterior after the block's
    // identity prediction; no exported full covariance needs materializing.
    if (core_->rtU.prediction_context[0] != 0.0 || std::any_of(
        core_->rtU.prediction_context + 1, core_->rtU.prediction_context + 257,
        [](double value) {return value != 0.0;}))
    {
      throw std::logic_error("Correction capture requires zero dt and process noise");
    }
    expose_internal_state();
  }

  void expose_internal_state()
  {
    // rtY precedes the block's prediction stage. Read its posterior data stores
    // directly instead of evaluating another zero-time prediction.
    read_generated_posterior(*core_, state_, covariance_);
    normalize_state_covariance(
      state_, covariance_, has_published_ ? &published_state_ : nullptr);
  }

  void install_state_covariance(const State & desired_state, const Covariance & desired_covariance)
  {
    state_ = desired_state;
    covariance_ = desired_covariance;
    normalize_state_covariance(
      state_, covariance_, has_published_ ? &published_state_ : nullptr);
    write_generated_posterior(*core_, state_, covariance_);
  }

  void predict_to(const rclcpp::Time & stamp)
  {
    double remaining = (stamp - filter_time_).seconds();
    while (remaining > 1e-9) {
      const double dt = std::min(remaining, max_prediction_dt_);
      clear_inputs();
      set_process_noise(dt);
      core_->rtU.prediction_context[0] = dt;
      core_->step();
      filter_time_ = filter_time_ + rclcpp::Duration::from_seconds(dt);
      remaining = (stamp - filter_time_).seconds();
    }
  }

  void compensate_imu_offset(Measurement & event)
  {
    const tf2::Vector3 omega(event.value[3], event.value[4], event.value[5]);
    const tf2::Vector3 offset(event.offset[0], event.offset[1], event.offset[2]);
    const double dt = last_imu_motion_stamp_ns_ == 0 ? 0.0 :
      (event.stamp.nanoseconds() - last_imu_motion_stamp_ns_) * 1e-9;
    const double denominator = imu_angular_acceleration_time_constant_ + dt;
    const double decay = imu_angular_acceleration_time_constant_ / denominator;
    const bool derivative_available = dt > 0.0 && dt <= max_prediction_dt_;
    tf2::Vector3 alpha(0, 0, 0);
    if (derivative_available) {
      alpha = decay * last_imu_alpha_ + (omega - last_imu_omega_) / denominator;
    }
    const tf2::Vector3 correction = alpha.cross(offset) + omega.cross(omega.cross(offset));
    for (int i = 0; i < 3; ++i) {event.value[6 + i] -= correction[i];}
    // Jacobian of corrected acceleration w.r.t. this sample's rate. The
    // off-diagonal rate/accel covariance must accompany lever-arm removal.
    double b[9]{};
    const double skew_r[9]{0, -offset.z(), offset.y(), offset.z(), 0, -offset.x(),
      -offset.y(), offset.x(), 0};
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        b[3 * i + j] = (derivative_available ? skew_r[3 * i + j] / denominator : 0.0) -
          (omega[i] * offset[j] + (i == j ? omega.dot(offset) : 0.0) - 2 * offset[i] * omega[j]);
      }
    }
    std::array<double, 9> rate_cov{}, extra{};
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {rate_cov[3 * i + j] = event.covariance[(3 + i) + 16 * (3 + j)];}
    }
    const double rate_trace = rate_cov[0] + rate_cov[4] + rate_cov[8];
    max_imu_rate_variance_ = std::max(max_imu_rate_variance_, rate_trace);
    // A filtered derivative has total noise gain <=2/tau. Use an isotropic
    // spectral bound for its history, including temporal correlations, rather
    // than repeatedly pretending overlapping rate differences are independent.
    const double history_variance = 4 * max_imu_rate_variance_ /
      (imu_angular_acceleration_time_constant_ * imu_angular_acceleration_time_constant_);
    for (int i = 0; i < 3; ++i) {extra[3 * i + i] = history_variance;}
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        double cross = 0.0, added = 0.0;
        for (int k = 0; k < 3; ++k) {
          cross += rate_cov[3 * i + k] * b[3 * j + k];
          for (int l = 0; l < 3; ++l) {
            added += b[3 * i + k] * rate_cov[3 * k + l] * b[3 * j + l] +
              skew_r[3 * i + k] * extra[3 * k + l] * skew_r[3 * j + l];
          }
        }
        event.covariance[(3 + i) + 16 * (6 + j)] = cross;
        event.covariance[(6 + j) + 16 * (3 + i)] = cross;
        event.covariance[(6 + i) + 16 * (6 + j)] += added;
      }
    }
    last_imu_omega_ = omega; last_imu_alpha_ = alpha;
    last_imu_motion_stamp_ns_ = event.stamp.nanoseconds();
  }

  bool measurement_within_gate(const Measurement & event) const
  {
    const auto sensor = static_cast<std::size_t>(event.sensor);
    // The initial sensor datum must bootstrap an unknown depth/velocity.
    if (last_accepted_stamp_ns_[sensor] == 0) {return true;}
    if (event.sensor == Sensor::Depth) {
      if (depth_rejection_threshold_ == 0.0 || depth_config_[2] == 0.0) {return true;}
      const double residual = event.value[0] - state_[2];
      const double variance = covariance_[2 + 16 * 2] + event.covariance[0];
      return variance > 0.0 && residual * residual <=
             depth_rejection_threshold_ * depth_rejection_threshold_ * variance;
    }
    if (event.sensor != Sensor::Dvl || dvl_rejection_threshold_ == 0.0) {return true;}
    const double rx = event.offset[0], ry = event.offset[1], rz = event.offset[2];
    const double omega_map[9]{0, rz, -ry, -rz, 0, rx, ry, -rx, 0};
    double h[48]{}, residual[3]{};
    int axes[3]{}, size = 0;
    for (int i = 0; i < 3; ++i) {
      if (dvl_config_[6 + i] == 0.0) {continue;}
      axes[size] = i;
      h[16 * size + 7 + i] = 1.0;
      residual[size] = event.value[i] - state_[7 + i];
      for (int j = 0; j < 3; ++j) {
        h[16 * size + 10 + j] = omega_map[3 * i + j];
        residual[size] -= omega_map[3 * i + j] * state_[10 + j];
      }
      ++size;
    }
    std::array<double, 81> innovation{}, factor{};
    for (int i = 0; i < size; ++i) {
      for (int j = 0; j < size; ++j) {
        double variance = event.covariance[axes[i] + 16 * axes[j]];
        for (int a = 7; a < 13; ++a) {
          for (int b = 7; b < 13; ++b) {
            variance += h[16 * i + a] * covariance_[a + 16 * b] * h[16 * j + b];
          }
        }
        innovation[9 * i + j] = variance;
      }
    }
    if (!innovation_factor(innovation, size, factor)) {return false;}
    double whitened[3]{}, squared = 0.0;
    for (int i = 0; i < size; ++i) {
      double value = residual[i];
      for (int j = 0; j < i; ++j) {value -= factor[9 * i + j] * whitened[j];}
      whitened[i] = value / factor[9 * i + i];
      squared += whitened[i] * whitened[i];
    }
    return std::isfinite(squared) && squared <=
           dvl_rejection_threshold_ * dvl_rejection_threshold_;
  }

  void process_measurement(const Measurement & input)
  {
    Measurement event = input;
    // A ROS clock using simulated time reads zero until its first /clock
    // update. Sensor stamps can then jump directly to an epoch-scale value.
    // Adopting the first observation's epoch avoids billions of bounded
    // prediction steps across time that never elapsed in the simulation.
    if (filter_time_.nanoseconds() == 0) {
      filter_time_ = event.stamp;
      ++startup_epoch_alignments_;
    }
    const std::size_t sensor_index = static_cast<std::size_t>(event.sensor);
    if (event.stamp.nanoseconds() <= last_processed_stamp_ns_[sensor_index]) {
      ++duplicate_rejections_;
      ++duplicate_by_sensor_[sensor_index];
      return;
    }
    if (event.stamp < filter_time_) {
      ++stale_rejections_;
      ++stale_by_sensor_[sensor_index];
      return;
    }
    if (event.sensor == Sensor::Imu) {
      if (event.imu_groups[1]) {
        compensate_imu_offset(event);
      } else {
        last_imu_motion_stamp_ns_ = 0;
        last_imu_omega_ = last_imu_alpha_ = tf2::Vector3(0, 0, 0);
      }
    }
    last_processed_stamp_ns_[sensor_index] = event.stamp.nanoseconds();
    processing_age_ms_[sensor_index] = (get_clock()->now() - event.stamp).seconds() * 1000.0;
    queue_residence_ms_[sensor_index] = event.arrival_stamp_ns == 0 ? 0.0 :
      (get_clock()->now().nanoseconds() - event.arrival_stamp_ns) * 1e-6;
    last_queue_age_ms_ = std::max(0.0, (get_clock()->now() - event.stamp).seconds() * 1000.0);
    predict_to(event.stamp);
    expose_internal_state();
    if (initial_tilt_pending_ && event.sensor == Sensor::Imu && event.imu_groups[0] &&
      imu_config_[3] != 0.0 && imu_config_[4] != 0.0)
    {
      const auto aligned = initialize_gravity_attitude(
        state_, {event.value[0], event.value[1], event.value[2]});
      const auto aligned_covariance = transport_quaternion_covariance(state_, aligned, covariance_);
      install_state_covariance(aligned, aligned_covariance);
      initial_tilt_pending_ = false;
      ++tilt_initializations_;
    }
    const State state_before = state_;
    bool apply_imu_tilt = false;
    if (event.sensor == Sensor::Imu && event.imu_groups[0] &&
      (imu_config_[3] != 0.0 || imu_config_[4] != 0.0 || imu_config_[5] != 0.0))
    {
      const std::int64_t period_ns = static_cast<std::int64_t>(
        vectornav_tilt_correction_frequency_ > 0.0 ?
        std::llround(1e9 / vectornav_tilt_correction_frequency_) : 0);
      apply_imu_tilt = !has_tilt_correction_stamp_ ||
        event.stamp.nanoseconds() - last_tilt_correction_stamp_ns_ >= period_ns;
      if (apply_imu_tilt) {
        last_tilt_correction_stamp_ns_ = event.stamp.nanoseconds();
        has_tilt_correction_stamp_ = true;
        ++tilt_corrections_;
      }
    }
    last_innovation_norm_[static_cast<std::size_t>(event.sensor)] =
      innovation_norm(event, apply_imu_tilt);
    if (!measurement_within_gate(event)) {
      ++innovation_rejections_;
      ++innovation_by_sensor_[sensor_index];
      return;
    }
    clear_inputs();
    set_process_noise(0.0);
    core_->rtU.prediction_context[0] = 0.0;
    switch (event.sensor) {
      case Sensor::Imu: {
          const auto tilt_mask = imu_tilt_mask();
          core_->rtU.enable_imu = true;
          for (int i = 0; i < 3; ++i) {
            core_->rtU.imu_measurement[i] = 0.0;
            core_->rtU.imu_measurement[3 + i] =
              event.value[3 + i] * imu_config_[9 + i] * event.imu_groups[1];
            core_->rtU.imu_measurement[6 + i] =
              event.value[6 + i] * imu_config_[12 + i] * event.imu_groups[2];
            core_->rtU.imu_context[i] = gravity_ * event.value[i];
            core_->rtU.imu_context[3 + i] = apply_imu_tilt ? tilt_mask[i] : 0.0;
            core_->rtU.imu_context[6 + i] = imu_config_[9 + i] * event.imu_groups[1];
            core_->rtU.imu_context[9 + i] = imu_config_[12 + i] * event.imu_groups[2];
          }
          copy_masked_covariance(event, 9, core_->rtU.imu_context + 3, core_->rtU.R_imu);
          break;
        }
      case Sensor::Fog:
        core_->rtU.enable_fog = true;
        for (int i = 0; i < 3; ++i) {
          core_->rtU.fog_mask[i] = fog_config_[9 + i];
          core_->rtU.fog_measurement[i] = event.value[i] * fog_config_[9 + i];
        }
        copy_masked_covariance(event, 3, core_->rtU.fog_mask, core_->rtU.R_fog);
        break;
      case Sensor::Dvl:
        core_->rtU.enable_dvl = true;
        for (int i = 0; i < 3; ++i) {
          core_->rtU.dvl_measurement[i] = event.value[i] * dvl_config_[6 + i];
          core_->rtU.dvl_context[i] = event.offset[i];
          core_->rtU.dvl_context[3 + i] = dvl_config_[6 + i];
        }
        copy_masked_covariance(event, 3, core_->rtU.dvl_context + 3, core_->rtU.R_dvl);
        break;
      case Sensor::Depth:
        core_->rtU.enable_depth = true;
        core_->rtU.depth_mask = depth_config_[2];
        core_->rtU.depth_measurement = event.value[0] * depth_config_[2];
        core_->rtU.R_depth = depth_config_[2] != 0.0 ? event.covariance[0] : 1.0;
        break;
      default: break;
    }
    if (event.sensor == Sensor::Imu && imu_acceleration_update_ == "consider") {
      State corrected = state_;
      Covariance corrected_covariance = covariance_;
      if (!correct_imu_consider_acceleration(
          corrected, corrected_covariance, core_->rtU.imu_measurement,
          core_->rtU.imu_context, core_->rtU.R_imu))
      {
        ++invalid_rejections_;
        ++invalid_by_sensor_[sensor_index];
        return;
      }
      state_ = corrected;
      covariance_ = corrected_covariance;
    } else if (event.sensor == Sensor::Fog && scalar_fog_correction_ &&
      std::count(fog_config_.begin() + 9, fog_config_.begin() + 12, 1.0) == 1)
    {
      const int axis = std::find(
        fog_config_.begin() + 9, fog_config_.begin() + 12, 1.0) - (fog_config_.begin() + 9);
      State corrected = state_;
      Covariance corrected_covariance = covariance_;
      if (!correct_scalar_observation(
          corrected, corrected_covariance, 10 + axis,
          event.value[axis], event.covariance[axis + 16 * axis]))
      {
        ++invalid_rejections_;
        ++invalid_by_sensor_[sensor_index];
        return;
      }
      // A single FOG axis is a linear observation. Avoid the block's unused
      // zero-time prediction/QR, retaining its full correlated Kalman update.
      install_state_covariance(corrected, corrected_covariance);
    } else {
      core_->step();
      capture_output();
    }
    filter_time_ = event.stamp;
    if (event.sensor == Sensor::Imu && apply_imu_tilt) {
      State projected = remove_world_yaw_correction(state_before, state_);
      const Covariance projected_covariance =
        transport_quaternion_covariance(state_, projected, covariance_);
      install_state_covariance(projected, projected_covariance);
    } else if (event.sensor == Sensor::Imu && imu_acceleration_update_ == "consider") {
      install_state_covariance(state_, covariance_);
    }
    double correction_squared = 0.0;
    for (int i = 0; i < 16; ++i) {
      const double delta = state_[i] - state_before[i];
      correction_squared += delta * delta;
    }
    last_correction_norm_[static_cast<std::size_t>(event.sensor)] =
      std::sqrt(correction_squared);
    ++accepted_[static_cast<std::size_t>(event.sensor)];
    last_accepted_stamp_ns_[sensor_index] = event.stamp.nanoseconds();
    initialized_ = true;
  }

  std::array<double, 3> imu_tilt_mask() const
  {
    std::array<double, 3> mask{{imu_config_[3], imu_config_[4], imu_config_[5]}};
    // Roll and pitch together select the two observable gravity degrees of
    // freedom. All three cross-product coordinates are needed at vertical
    // attitudes: dropping body Z loses a tilt direction there. Gravity is
    // intrinsically insensitive to world yaw, even with all rows enabled.
    if (mask[0] != 0.0 && mask[1] != 0.0) {mask.fill(1.0);}
    return mask;
  }

  double innovation_norm(const Measurement & event, bool apply_imu_tilt = true) const
  {
    std::array<double, 9> residual{};
    int size = 1;
    switch (event.sensor) {
      case Sensor::Imu: {
          const auto r = rotation(state_);
          const tf2::Vector3 measured_gravity(event.value[0], event.value[1], event.value[2]);
          const tf2::Vector3 predicted_gravity(r[6], r[7], r[8]);
          const tf2::Vector3 tilt = measured_gravity.cross(predicted_gravity);
          const auto tilt_mask = imu_tilt_mask();
          residual[0] = -tilt.x() * tilt_mask[0] * apply_imu_tilt;
          residual[1] = -tilt.y() * tilt_mask[1] * apply_imu_tilt;
          residual[2] = -tilt.z() * tilt_mask[2] * apply_imu_tilt;
          for (int i = 0; i < 3; ++i) {
            residual[3 + i] = (event.value[3 + i] - state_[10 + i]) *
              imu_config_[9 + i] * event.imu_groups[1];
            residual[6 + i] = (event.value[6 + i] - state_[13 + i] - gravity_ * r[6 + i]) *
              imu_config_[12 + i] * event.imu_groups[2];
          }
          size = 9;
          break;
        }
      case Sensor::Fog:
        for (int i = 0; i < 3; ++i) {
          residual[i] = (event.value[i] - state_[10 + i]) * fog_config_[9 + i];
        }
        size = 3;
        break;
      case Sensor::Dvl: {
          const tf2::Vector3 omega(state_[10], state_[11], state_[12]);
          const tf2::Vector3 offset(event.offset[0], event.offset[1], event.offset[2]);
          const tf2::Vector3 sensor_velocity =
            tf2::Vector3(state_[7], state_[8], state_[9]) + omega.cross(offset);
          residual[0] = (event.value[0] - sensor_velocity.x()) * dvl_config_[6];
          residual[1] = (event.value[1] - sensor_velocity.y()) * dvl_config_[7];
          residual[2] = (event.value[2] - sensor_velocity.z()) * dvl_config_[8];
          size = 3;
          break;
        }
      case Sensor::Depth:
        residual[0] = (event.value[0] - state_[2]) * depth_config_[2];
        break;
      default:
        return 0.0;
    }
    double squared = 0.0;
    for (int i = 0; i < size; ++i) {
      squared += residual[i] * residual[i];
    }
    return std::sqrt(squared);
  }

  Covariance reset_covariance(
    const State & state, const geometry_msgs::msg::PoseWithCovarianceStamped * pose = nullptr)
  {
    Covariance result{};
    for (int i = 0; i < 16; ++i) {result[i + 16 * i] = reset_covariance_diag_[i];}
    if (pose == nullptr) {return result;}
    const bool covariance_specified = std::any_of(
      pose->pose.covariance.begin(), pose->pose.covariance.end(),
      [](double value) {return value != 0.0;});
    if (!covariance_specified) {return result;}
    return apply_ros_pose_covariance(state, pose->pose.covariance, result);
  }

  void reset_filter(
    State state, const rclcpp::Time & stamp,
    const geometry_msgs::msg::PoseWithCovarianceStamped * pose = nullptr)
  {
    clear_preview_cache();
    // An explicit reset supplies the intended pose and must not be overridden
    // by startup initialization on the next sensor observation.
    initial_tilt_pending_ = false;
    while (!queue_.empty()) {queue_.pop();}
    last_processed_stamp_ns_.fill(0);
    last_accepted_stamp_ns_.fill(0);
    last_imu_motion_stamp_ns_ = 0;
    max_imu_rate_variance_ = 0.0;
    has_tilt_correction_stamp_ = false;
    last_tilt_correction_stamp_ns_ = 0;
    normalize_quaternion(state, has_published_ ? &published_state_ : nullptr);
    core_ = std::make_unique<talos_ekf>();
    core_->initialize();
    const Covariance desired_covariance = reset_covariance(state, pose);
    install_state_covariance(state, desired_covariance);
    filter_time_ = stamp;
    capture_output();
    initialized_ = true;
    ++resets_;
    RCLCPP_INFO(
      get_logger(), "Reset EKF at %.3f s to position [%.3f, %.3f, %.3f]",
      stamp.seconds(), state_[0], state_[1], state_[2]);
  }

  std::optional<State> pose_state(
    const geometry_msgs::msg::PoseWithCovarianceStamped & msg,
    geometry_msgs::msg::PoseWithCovarianceStamped & transformed)
  {
    transformed = msg;
    if (!msg.header.frame_id.empty() && msg.header.frame_id != world_frame_) {
      try {
        tf2::doTransform(
          msg, transformed,
          tf_buffer_.lookupTransform(
            world_frame_, msg.header.frame_id,
            rclcpp::Time(msg.header.stamp, get_clock()->get_clock_type())));
      } catch (const tf2::TransformException &) {
        ++transform_rejections_;
        return std::nullopt;
      }
    }
    Covariance check{};
    for (int i = 0; i < 6; ++i) {
      for (int j = 0; j < 6; ++j) {
        const double value = transformed.pose.covariance[6 * i + j];
        if (!std::isfinite(value) ||
          std::abs(value - transformed.pose.covariance[6 * j + i]) > 1e-9)
        {
          ++invalid_rejections_;
          return std::nullopt;
        }
        check[i + 16 * j] = value;
      }
    }
    Covariance root{};
    if (!posterior_square_root(check, root)) {++invalid_rejections_; return std::nullopt;}
    const auto & pose = transformed.pose.pose;
    State result{};
    result[0] = pose.position.x; result[1] = pose.position.y; result[2] = pose.position.z;
    result[3] = pose.orientation.w; result[4] = pose.orientation.x;
    result[5] = pose.orientation.y; result[6] = pose.orientation.z;
    if (!finite(result) || !normalize_quaternion(result)) {
      ++invalid_rejections_;
      return std::nullopt;
    }
    return result;
  }

  void set_pose_topic(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(filter_mutex_);
    geometry_msgs::msg::PoseWithCovarianceStamped transformed;
    if (auto requested = pose_state(*msg, transformed)) {
      reset_filter(*requested, get_clock()->now(), &transformed);
    }
  }

  void set_pose_service(
    const std::shared_ptr<robot_localization::srv::SetPose::Request> request,
    std::shared_ptr<robot_localization::srv::SetPose::Response>)
  {
    std::lock_guard<std::mutex> lock(filter_mutex_);
    geometry_msgs::msg::PoseWithCovarianceStamped transformed;
    if (auto requested = pose_state(request->pose, transformed)) {
      reset_filter(*requested, get_clock()->now(), &transformed);
    }
  }

  void fill_pose_covariance(nav_msgs::msg::Odometry & msg, const Covariance & p, const State & x)
  {
    msg.pose.covariance = ros_pose_covariance(x, p);
  }

  auto checkpoint_fields()
  {
    return std::tie(
      state_, covariance_, filter_time_, last_processed_stamp_ns_,
      last_imu_motion_stamp_ns_, last_imu_omega_, last_imu_alpha_,
      last_tilt_correction_stamp_ns_, has_tilt_correction_stamp_, tilt_corrections_,
      max_imu_rate_variance_, accepted_, stale_rejections_, duplicate_rejections_,
      invalid_rejections_, stale_by_sensor_, duplicate_by_sensor_, invalid_by_sensor_,
      processing_age_ms_, queue_residence_ms_, last_queue_age_ms_,
      last_innovation_norm_, last_correction_norm_, startup_epoch_alignments_,
      initial_tilt_pending_, tilt_initializations_, last_accepted_stamp_ns_,
      innovation_rejections_, innovation_by_sensor_, initialized_);
  }

  void clear_preview_cache()
  {
    cached_restore_ = {};
    cached_sequences_.clear();
    cached_processing_stamps_.fill(0);
    cached_last_processing_stamp_ns_ = 0;
  }

  bool preview_queued_updates(
    const rclcpp::Time & stamp,
    State & output, Covariance & output_covariance, rclcpp::Time & output_time)
  {
    // Keep the delayed filter authoritative. A matching preview prefix can
    // later become authoritative when it is entirely behind the watermark.
    // Invalidate the previous preview even when this publication has no work.
    clear_preview_cache();
    const auto preview_start = std::chrono::steady_clock::now();
    auto fields = checkpoint_fields();
    const auto base_state = state_;
    const auto base_covariance = covariance_;
    const auto base_time_ns = filter_time_.nanoseconds();
    const bool base_initialized = initialized_;
    bool output_ready = initialized_;
    const auto invalid_baseline = invalid_rejections_;
    const auto invalid_by_sensor_baseline = invalid_by_sensor_;
    const auto checkpoint = std::apply(
      [](const auto & ... values) {
        return std::make_tuple(values ...);
      }, fields);
    if (!preview_core_) {
      preview_core_ = std::make_unique<talos_ekf>();
      preview_core_->initialize();
    }
    std::swap(core_, preview_core_);
    try {
      copy_generated_core_state(*core_, *preview_core_);
      auto pending = queue_;
      std::vector<std::uint64_t> sequences;
      std::int64_t prefix_last_stamp_ns = 0;
      preview_events_ = 0;
      while (!pending.empty() && pending.top().stamp <= stamp &&
        preview_events_ < max_measurements_per_cycle_)
      {
        const Measurement event = pending.top();
        pending.pop();
        sequences.push_back(event.sequence);
        prefix_last_stamp_ns = event.stamp.nanoseconds();
        const auto sensor = static_cast<std::size_t>(event.sensor);
        const auto previous_stamp = last_processed_stamp_ns_[sensor];
        process_measurement(event);
        if (last_processed_stamp_ns_[sensor] != previous_stamp) {
          cached_processing_stamps_[sensor] = event.stamp.nanoseconds();
          cached_arrival_stamps_[sensor] = event.arrival_stamp_ns;
          cached_last_processing_stamp_ns_ = event.stamp.nanoseconds();
        }
        ++preview_events_;
      }
      preview_limited_ = !pending.empty() && pending.top().stamp <= stamp;
      if (reuse_preview_corrections_ && !sequences.empty()) {
        const auto saved = std::apply(
          [](const auto & ... values) {
            return std::make_tuple(values ...);
          }, fields);
        cached_restore_ = [fields, saved]() mutable {fields = saved;};
        if (!cached_core_) {cached_core_ = std::make_unique<talos_ekf>();}
        copy_generated_core_state(*cached_core_, *core_);
        cached_sequences_ = std::move(sequences);
        cached_prefix_last_stamp_ns_ = prefix_last_stamp_ns;
        cached_base_time_ns_ = base_time_ns;
        cached_base_initialized_ = base_initialized;
        cached_base_state_ = base_state;
        cached_base_covariance_ = base_covariance;
        cached_invalid_baseline_ = invalid_baseline;
        cached_invalid_by_sensor_baseline_ = invalid_by_sensor_baseline;
      }
      // Use the same generated kinematics/Jacobian/Q as the authoritative
      // filter for the publication horizon. The checkpoint keeps this
      // prediction isolated from acceptance of later sensor arrivals.
      publication_prediction_ms_ = std::max(0.0, (stamp - filter_time_).seconds()) * 1000.0;
      output_ready = initialized_;
      if (output_ready) {predict_to(stamp); expose_internal_state();}
      output = state_;
      output_covariance = covariance_;
      output_time = filter_time_;
    } catch (...) {
      clear_preview_cache();
      fields = checkpoint;
      std::swap(core_, preview_core_);
      throw;
    }
    fields = checkpoint;
    std::swap(core_, preview_core_);
    preview_processing_ms_ = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - preview_start).count();
    return output_ready;
  }

  bool publish(const rclcpp::Time & stamp)
  {
    State output = state_;
    Covariance output_covariance = covariance_;
    rclcpp::Time output_time = filter_time_;
    if (publication_mode_ == "queued_updates") {
      if (!preview_queued_updates(stamp, output, output_covariance, output_time)) {return false;}
    } else if (!initialized_) {
      return false;
    }
    double remaining = std::max(0.0, (stamp - output_time).seconds());
    if (publication_mode_ != "queued_updates") {publication_prediction_ms_ = remaining * 1000.0;}
    while (remaining > 1e-9) {
      const double dt = std::min(remaining, max_prediction_dt_);
      output_covariance = predict_covariance(output, output_covariance, dt, process_noise_diag_);
      output = predict_state(output, dt);
      remaining -= dt;
    }
    normalize_state_covariance(
      output, output_covariance,
      has_published_ ? &published_state_ : nullptr);
    published_state_ = output;
    has_published_ = true;

    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = stamp; odometry.header.frame_id = world_frame_;
    odometry.child_frame_id = base_frame_;
    odometry.pose.pose.position.x = output[0]; odometry.pose.pose.position.y = output[1];
    odometry.pose.pose.position.z = output[2];
    odometry.pose.pose.orientation.w = output[3]; odometry.pose.pose.orientation.x = output[4];
    odometry.pose.pose.orientation.y = output[5]; odometry.pose.pose.orientation.z = output[6];
    odometry.twist.twist.linear.x = output[7]; odometry.twist.twist.linear.y = output[8];
    odometry.twist.twist.linear.z = output[9];
    odometry.twist.twist.angular.x = output[10]; odometry.twist.twist.angular.y = output[11];
    odometry.twist.twist.angular.z = output[12];
    fill_pose_covariance(odometry, output_covariance, output);
    for (int row = 0; row < 6; ++row) {
      for (int col = 0; col < 6; ++col) {
        odometry.twist.covariance[row * 6 + col] =
          output_covariance[(7 + row) + 16 * (7 + col)];
      }
    }
    odometry_pub_->publish(odometry);

    geometry_msgs::msg::AccelWithCovarianceStamped acceleration;
    acceleration.header = odometry.header;
    acceleration.header.frame_id = base_frame_;
    acceleration.accel.accel.linear.x = output[13]; acceleration.accel.accel.linear.y = output[14];
    acceleration.accel.accel.linear.z = output[15];
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        acceleration.accel.covariance[row * 6 + col] =
          output_covariance[(13 + row) + 16 * (13 + col)];
      }
    }
    acceleration_pub_->publish(acceleration);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = odometry.header; transform.child_frame_id = base_frame_;
      transform.transform.translation.x = output[0]; transform.transform.translation.y = output[1];
      transform.transform.translation.z = output[2];
      transform.transform.rotation = odometry.pose.pose.orientation;
      tf_broadcaster_->sendTransform(transform);
    }
    return true;
  }

  std::string timed_out_sensors(const rclcpp::Time & stamp) const
  {
    const bool enabled[]{
      std::any_of(imu_config_.begin(), imu_config_.end(), [](double x) {return x != 0.0;}),
      std::any_of(fog_config_.begin(), fog_config_.end(), [](double x) {return x != 0.0;}),
      std::any_of(dvl_config_.begin(), dvl_config_.end(), [](double x) {return x != 0.0;}),
      std::any_of(depth_config_.begin(), depth_config_.end(), [](double x) {return x != 0.0;})};
    static const char * names[]{"imu", "fog", "dvl", "depth"};
    std::string result;
    for (std::size_t i = 0; i < 4; ++i) {
      if (enabled[i] && (last_accepted_stamp_ns_[i] == 0 ||
        (stamp.nanoseconds() - last_accepted_stamp_ns_[i]) * 1e-9 > sensor_timeouts_[i]))
      {
        if (!result.empty()) {result += ", ";}
        result += names[i];
      }
    }
    return result;
  }

  void publish_diagnostics(const rclcpp::Time & stamp)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = stamp;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = get_fully_qualified_name() + std::string(": estimator");
    status.hardware_id = "talos";
    const bool valid = finite(state_) && covariance_is_psd(covariance_);
    status.level =
      valid ? diagnostic_msgs::msg::DiagnosticStatus::OK : diagnostic_msgs::msg::DiagnosticStatus
      ::ERROR;
    status.message = valid ? "Quaternion EKF active" : "Invalid state or covariance";
    const auto missing = timed_out_sensors(stamp);
    if (valid && !initialized_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "Waiting for an accepted measurement or pose reset";
    } else if (valid && !missing.empty()) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "Sensor timeout: " + missing;
    }
    const char * group_names[]{"orientation", "angular_velocity", "acceleration"};
    for (int group = 0; group < 3; ++group) {
      status.values.push_back(
        key_value(
          std::string("imu_") + group_names[group] + "_available",
          latest_imu_groups_[group] ? 1.0 : 0.0));
      status.values.push_back(
        key_value(
          std::string("imu_") + group_names[group] + "_unavailable_samples",
          imu_unavailable_groups_[group]));
      const int first = group == 0 ? 3 : group == 1 ? 9 : 12;
      const bool enabled = imu_config_[first] != 0.0 || imu_config_[first + 1] != 0.0 ||
        imu_config_[first + 2] != 0.0;
      if (valid && enabled && !latest_imu_groups_[group]) {
        if (status.level == diagnostic_msgs::msg::DiagnosticStatus::OK) {
          status.message = "IMU group unavailable:";
        }
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message += std::string(" ") + group_names[group];
      }
    }
    status.values.push_back(key_value("preview_promotions", preview_promotions_));
    status.values.push_back(key_value("preview_cache_misses", preview_cache_misses_));
    status.values.push_back(key_value("reused_measurements", reused_measurements_));
    status.values.push_back(key_value("queue_depth", queue_.size()));
    status.values.push_back(key_value("implementation_version", kImplementationVersion));
    status.values.push_back(key_value("imu_acceleration_update", imu_acceleration_update_));
    status.values.push_back(key_value("publication_mode", publication_mode_));
    status.values.push_back(key_value("publication_prediction_ms", publication_prediction_ms_));
    status.values.push_back(key_value("publication_preview_events", preview_events_));
    status.values.push_back(key_value("publication_preview_processing_ms", preview_processing_ms_));
    status.values.push_back(key_value("publication_preview_limited", preview_limited_ ? 1.0 : 0.0));
    status.values.push_back(
      key_value("vectornav_tilt_correction_frequency_hz", vectornav_tilt_correction_frequency_));
    status.values.push_back(key_value("tilt_corrections", tilt_corrections_));
    status.values.push_back(key_value("tilt_initializations", tilt_initializations_));
    status.values.push_back(key_value("resets", resets_));
    status.values.push_back(key_value("stale_rejections", stale_rejections_));
    status.values.push_back(key_value("transform_rejections", transform_rejections_));
    status.values.push_back(key_value("invalid_rejections", invalid_rejections_));
    status.values.push_back(key_value("innovation_rejections", innovation_rejections_));
    status.values.push_back(key_value("queue_rejections", queue_rejections_));
    status.values.push_back(key_value("duplicate_rejections", duplicate_rejections_));
    status.values.push_back(key_value("startup_epoch_alignments", startup_epoch_alignments_));
    status.values.push_back(key_value("imu_accepted", accepted_[0]));
    status.values.push_back(key_value("fog_accepted", accepted_[1]));
    status.values.push_back(key_value("dvl_accepted", accepted_[2]));
    status.values.push_back(key_value("depth_accepted", accepted_[3]));
    static const char * sensor_names[]{"imu", "fog", "dvl", "depth"};
    for (std::size_t i = 0; i < accepted_.size(); ++i) {
      const std::string prefix = std::string(sensor_names[i]) + "_";
      status.values.push_back(key_value(prefix + "stale_rejections", stale_by_sensor_[i]));
      status.values.push_back(key_value(prefix + "duplicate_rejections", duplicate_by_sensor_[i]));
      status.values.push_back(key_value(prefix + "transform_rejections", transform_by_sensor_[i]));
      status.values.push_back(key_value(prefix + "invalid_rejections", invalid_by_sensor_[i]));
      status.values.push_back(key_value(prefix + "queue_rejections", queue_by_sensor_[i]));
      status.values.push_back(
        key_value(
          prefix + "innovation_rejections",
          innovation_by_sensor_[i]));
      status.values.push_back(key_value(prefix + "arrival_age_ms", arrival_age_ms_[i]));
      status.values.push_back(key_value(prefix + "min_arrival_age_ms", min_arrival_age_ms_[i]));
      status.values.push_back(key_value(prefix + "max_arrival_age_ms", max_arrival_age_ms_[i]));
      status.values.push_back(key_value(prefix + "processing_age_ms", processing_age_ms_[i]));
      status.values.push_back(key_value(prefix + "queue_residence_ms", queue_residence_ms_[i]));
      status.values.push_back(
        key_value(
          prefix + "measurement_age_ms",
          last_accepted_stamp_ns_[i] == 0 ? -1.0 :
          (stamp.nanoseconds() - last_accepted_stamp_ns_[i]) * 1e-6));
      status.values.push_back(key_value(prefix + "timeout_ms", sensor_timeouts_[i] * 1000.0));
    }
    status.values.push_back(key_value("processing_duration_ms", processing_duration_ms_));
    status.values.push_back(key_value("max_processing_duration_ms", max_processing_duration_ms_));
    status.values.push_back(
      key_value(
        "publication_completion_age_ms",
        publication_completion_age_ms_));
    if (publish_debug_diagnostics_) {
      status.values.push_back(key_value("queue_age_ms", last_queue_age_ms_));
      status.values.push_back(
        key_value(
          "filter_lag_ms", std::max(0.0, (stamp - filter_time_).seconds() * 1000.0)));
      static const char * names[] = {"imu", "fog", "dvl", "depth"};
      for (std::size_t i = 0; i < accepted_.size(); ++i) {
        status.values.push_back(
          key_value(
            std::string(names[i]) + "_innovation_norm", last_innovation_norm_[i]));
        status.values.push_back(
          key_value(
            std::string(names[i]) + "_correction_norm", last_correction_norm_[i]));
      }
    }
    array.status.push_back(std::move(status));
    diagnostics_pub_->publish(array);
  }

  std::size_t reuse_preview_cache(const rclcpp::Time & watermark)
  {
    if (!cached_restore_) {return 0;}
    bool matches = reuse_preview_corrections_ &&
      cached_prefix_last_stamp_ns_ <= watermark.nanoseconds() &&
      cached_sequences_.size() <= max_measurements_per_cycle_ &&
      filter_time_.nanoseconds() == cached_base_time_ns_ &&
      state_ == cached_base_state_ && covariance_ == cached_base_covariance_ &&
      initialized_ == cached_base_initialized_;
    auto pending = queue_;
    if (matches) {
      for (auto sequence : cached_sequences_) {
        if (pending.empty() || pending.top().sequence != sequence) {matches = false; break;}
        pending.pop();
      }
    }
    std::size_t reused = 0;
    if (matches) {
      // Invalid callbacks/SetPose requests can arrive after the preview and
      // before promotion. Keep their counters while restoring filter fields.
      const auto invalid_delta = invalid_rejections_ - cached_invalid_baseline_;
      auto invalid_by_sensor_delta = invalid_by_sensor_;
      for (std::size_t i = 0; i < invalid_by_sensor_delta.size(); ++i) {
        invalid_by_sensor_delta[i] -= cached_invalid_by_sensor_baseline_[i];
      }
      copy_generated_core_state(*core_, *cached_core_);
      cached_restore_();
      invalid_rejections_ += invalid_delta;
      const auto now_ns = get_clock()->now().nanoseconds();
      for (std::size_t i = 0; i < invalid_by_sensor_delta.size(); ++i) {
        invalid_by_sensor_[i] += invalid_by_sensor_delta[i];
        if (cached_processing_stamps_[i] != 0) {
          processing_age_ms_[i] = (now_ns - cached_processing_stamps_[i]) * 1e-6;
          queue_residence_ms_[i] = cached_arrival_stamps_[i] == 0 ? 0.0 :
            (now_ns - cached_arrival_stamps_[i]) * 1e-6;
        }
      }
      if (cached_last_processing_stamp_ns_ != 0) {
        last_queue_age_ms_ = std::max(0.0, (now_ns - cached_last_processing_stamp_ns_) * 1e-6);
      }
      reused = cached_sequences_.size();
      queue_ = std::move(pending);
      ++preview_promotions_;
      reused_measurements_ += reused;
    } else {++preview_cache_misses_;}
    clear_preview_cache();
    return reused;
  }

  void timer_callback()
  {
    std::lock_guard<std::mutex> lock(filter_mutex_);
    const auto processing_start = std::chrono::steady_clock::now();
    const rclcpp::Time now = get_clock()->now();
    if (filter_time_.nanoseconds() == 0 && now.nanoseconds() > 0) {
      filter_time_ = queue_.empty() ? now : std::min(now, queue_.top().stamp);
      ++startup_epoch_alignments_;
    }
    if (last_clock_.nanoseconds() != 0 && now < last_clock_) {
      const bool was_initialized = initialized_;
      reset_filter(state_, now);
      initialized_ = was_initialized;
      last_publish_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      last_diagnostics_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    }
    last_clock_ = now;
    const rclcpp::Time watermark = now - rclcpp::Duration::from_seconds(
      std::max(
        0.0,
        reorder_delay_));
    std::size_t processed = reuse_preview_cache(watermark);
    while (!queue_.empty() && queue_.top().stamp <= watermark &&
      processed < max_measurements_per_cycle_)
    {
      Measurement event = queue_.top(); queue_.pop(); process_measurement(event);
      ++processed;
    }
    processing_duration_ms_ = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - processing_start).count();
    max_processing_duration_ms_ = std::max(max_processing_duration_ms_, processing_duration_ms_);
    const rclcpp::Time publication_stamp = get_clock()->now();
    const bool preview_ready = publication_mode_ == "queued_updates" &&
      !queue_.empty() && queue_.top().stamp <= publication_stamp;
    if ((initialized_ || preview_ready) &&
      (last_publish_.nanoseconds() == 0 || publication_stamp > last_publish_))
    {
      if (publish(publication_stamp)) {last_publish_ = publication_stamp;}
      publication_completion_age_ms_ = (get_clock()->now() - publication_stamp).seconds() * 1000.0;
    }
    if (last_diagnostics_.nanoseconds() == 0 || (now - last_diagnostics_).seconds() >= 1.0) {
      publish_diagnostics(now); last_diagnostics_ = now;
    }
  }

  std::string world_frame_, base_frame_, imu_orientation_covariance_coordinates_;
  bool publish_tf_{}, publish_debug_diagnostics_{};
  bool initialize_tilt_from_imu_{}, initial_tilt_pending_{};
  std::array<bool, 3> latest_imu_groups_{{true, true, true}};
  std::array<std::uint64_t, 3> imu_unavailable_groups_{};
  std::string imu_acceleration_update_, publication_mode_;
  std::unique_ptr<talos_ekf> preview_core_;
  bool reuse_preview_corrections_{};
  bool scalar_fog_correction_{};
  std::unique_ptr<talos_ekf> cached_core_;
  std::function<void()> cached_restore_;
  std::vector<std::uint64_t> cached_sequences_;
  bool cached_base_initialized_{};
  State cached_base_state_{};
  Covariance cached_base_covariance_{};
  std::int64_t cached_base_time_ns_{}, cached_prefix_last_stamp_ns_{};
  std::array<std::int64_t, 4> cached_processing_stamps_{}, cached_arrival_stamps_{};
  std::int64_t cached_last_processing_stamp_ns_{};
  std::uint64_t cached_invalid_baseline_{};
  std::array<std::uint64_t, 4> cached_invalid_by_sensor_baseline_{};
  std::uint64_t preview_promotions_{}, preview_cache_misses_{}, reused_measurements_{};
  std::size_t preview_events_{};
  bool preview_limited_{};
  double publication_prediction_ms_{};
  double preview_processing_ms_{};
  double frequency_{}, vectornav_tilt_correction_frequency_{}, gravity_{}, reorder_delay_{};
  double max_prediction_dt_{}, imu_angular_acceleration_time_constant_{};
  std::int64_t last_imu_motion_stamp_ns_{};
  tf2::Vector3 last_imu_omega_{0, 0, 0}, last_imu_alpha_{0, 0, 0};
  double max_imu_rate_variance_{};
  std::size_t max_queue_size_{}, max_measurements_per_cycle_{};
  std::vector<double> process_noise_diag_, reset_covariance_diag_, sensor_timeouts_;
  std::array<double, kConfigSize> imu_config_{}, dvl_config_{}, fog_config_{}, depth_config_{};
  double default_orientation_variance_{}, default_angular_velocity_variance_{};
  double default_acceleration_variance_{}, default_fog_variance_{}, default_dvl_variance_{},
    default_depth_variance_{};
  double imu_covariance_multiplier_{}, fog_covariance_multiplier_{}, dvl_covariance_multiplier_{};
  double imu_orientation_covariance_multiplier_{}, imu_angular_velocity_covariance_multiplier_{};
  double imu_acceleration_covariance_multiplier_{};
  double depth_covariance_multiplier_{}, covariance_floor_{};
  double dvl_rejection_threshold_{}, depth_rejection_threshold_{};
  std::unique_ptr<talos_ekf> core_;
  State state_{}, published_state_{};
  Covariance covariance_{};
  bool initialized_{false}, has_published_{false};
  rclcpp::Time filter_time_{0, 0, RCL_ROS_TIME}, last_clock_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_publish_{0, 0, RCL_ROS_TIME}, last_diagnostics_{0, 0, RCL_ROS_TIME};
  std::priority_queue<Measurement, std::vector<Measurement>, EarlierMeasurement> queue_;
  std::uint64_t sequence_{}, resets_{}, stale_rejections_{}, transform_rejections_{};
  std::uint64_t invalid_rejections_{}, queue_rejections_{}, duplicate_rejections_{};
  std::uint64_t innovation_rejections_{};
  std::uint64_t startup_epoch_alignments_{};
  std::uint64_t tilt_corrections_{}, tilt_initializations_{};
  std::array<std::uint64_t, static_cast<std::size_t>(Sensor::Count)> accepted_{};
  std::array<std::int64_t, static_cast<std::size_t>(Sensor::Count)> last_processed_stamp_ns_{};
  std::array<std::int64_t, static_cast<std::size_t>(Sensor::Count)> last_accepted_stamp_ns_{};
  std::array<std::uint64_t, static_cast<std::size_t>(Sensor::Count)> innovation_by_sensor_{};
  std::int64_t last_tilt_correction_stamp_ns_{};
  bool has_tilt_correction_stamp_{false};
  std::array<double, static_cast<std::size_t>(Sensor::Count)> last_innovation_norm_{};
  std::array<double, static_cast<std::size_t>(Sensor::Count)> last_correction_norm_{};
  std::array<std::uint64_t, 4> arrival_counts_{};
  std::array<std::uint64_t, 4> stale_by_sensor_{}, duplicate_by_sensor_{}, transform_by_sensor_{},
    invalid_by_sensor_{}, queue_by_sensor_{};
  std::array<double, 4> arrival_age_ms_{}, processing_age_ms_{}, queue_residence_ms_{},
    min_arrival_age_ms_{}, max_arrival_age_ms_{};
  double processing_duration_ms_{}, max_processing_duration_ms_{}, publication_completion_age_ms_{};
  double last_queue_age_ms_{};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::CallbackGroup::SharedPtr reset_callback_group_;
  std::mutex filter_mutex_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr fog_sub_,
    dvl_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr depth_sub_,
    set_pose_sub_;
  rclcpp::Service<robot_localization::srv::SetPose>::SharedPtr set_pose_service_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr acceleration_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace riptide_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<riptide_navigation::TalosEkfNode>();
  // Sensor/timer callbacks share a mutually exclusive group, and reset also
  // takes filter_mutex_. TF and /clock already have dedicated listener threads.
  // Another executor worker adds contention without parallel estimator work.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
