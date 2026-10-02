// Copyright 2026 OSU Underwater Robotics Team
// SPDX-License-Identifier: BSD-3-Clause
#include "riptide_navigation/estimator_support.hpp"
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
  std::uint64_t sequence{};
};

constexpr std::size_t kConfigSize = 15;

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

}  // namespace

class TalosEkfNode final : public rclcpp::Node
{
public:
  TalosEkfNode()
  : Node("talos_ekf_node"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    world_frame_ = declare_parameter("world_frame", "odom");
    base_frame_ = declare_parameter("base_link_frame", "talos/base_link");
    publish_tf_ = declare_parameter("publish_tf", true);
    frequency_ = declare_parameter("frequency", 30.0);
    gravity_ = declare_parameter("gravitational_acceleration", 9.755455);
    reorder_delay_ = declare_parameter("reorder_delay", 0.02);
    max_queue_size_ = static_cast<std::size_t>(declare_parameter("max_queue_size", 512));
    max_measurements_per_cycle_ = static_cast<std::size_t>(
      declare_parameter("max_measurements_per_cycle", 64));
    max_prediction_dt_ = declare_parameter("max_prediction_dt", 0.1);
    process_noise_diag_ = declare_parameter<std::vector<double>>(
      "process_noise_diag", {5e-5, 5e-5, 6e-5, 3e-5, 3e-5, 6e-5, 1e-6, 2.5e-5,
        2.5e-5, 4e-5, 1e-5, 1e-5, 2e-5, 1e-5, 1e-5, 1.5e-5});
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
    fog_covariance_multiplier_ = declare_parameter("fog_covariance_multiplier", 1.0);
    dvl_covariance_multiplier_ = declare_parameter("dvl_covariance_multiplier", 1.0);
    depth_covariance_multiplier_ = declare_parameter("depth_covariance_multiplier", 1.0);
    covariance_floor_ = declare_parameter("covariance_floor", 1e-9);
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
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, frequency_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&TalosEkfNode::timer_callback, this));
    State initial{};
    initial[3] = 1.0;
    reset_filter(initial, get_clock()->now());
  }

private:
  std::optional<geometry_msgs::msg::TransformStamped> sensor_transform(
    const std::string & frame, const rclcpp::Time & stamp)
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
    if (!valid_stamp(measurement.stamp)) {return;}
    const bool finite_measurement = std::all_of(
      measurement.value.begin(), measurement.value.end(),
      [](double value) {return std::isfinite(value);}) && std::all_of(
      measurement.covariance.begin(), measurement.covariance.end(),
      [](double value) {return std::isfinite(value);}) && std::all_of(
      measurement.offset.begin(), measurement.offset.end(),
      [](double value) {return std::isfinite(value);});
    if (!finite_measurement) {
      ++invalid_rejections_;
      return;
    }
    if (queue_.size() >= max_queue_size_) {
      ++queue_rejections_;
      return;
    }
    measurement.sequence = sequence_++;
    queue_.push(std::move(measurement));
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
    auto transform = sensor_transform(msg->header.frame_id, stamp);
    if (!transform) {return;}
    tf2::Quaternion q_bs;
    tf2::fromMsg(transform->transform.rotation, q_bs);
    q_bs.normalize();
    tf2::Quaternion q_ws(msg->orientation.x, msg->orientation.y, msg->orientation.z,
      msg->orientation.w);
    if (q_ws.length2() < 1e-12 || !std::isfinite(q_ws.length2())) {
      ++invalid_rejections_;
      return;
    }
    q_ws.normalize();
    tf2::Quaternion q_wb = q_ws * q_bs.inverse();
    q_wb.normalize();
    const tf2::Vector3 gravity_body = tf2::quatRotate(q_wb.inverse(), tf2::Vector3(0, 0, 1));
    const tf2::Vector3 omega = tf2::quatRotate(
      q_bs,
      tf2::Vector3(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z));
    tf2::Vector3 accel = tf2::quatRotate(
      q_bs,
      tf2::Vector3(
        msg->linear_acceleration.x, msg->linear_acceleration.y,
        msg->linear_acceleration.z));
    accel -= gravity_ * gravity_body;

    Measurement event{stamp, Sensor::Imu};
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
    orientation = rotate_covariance(r_bs, orientation);
    angular = rotate_covariance(r_bs, angular);
    acceleration = rotate_covariance(r_bs, acceleration);
    condition_covariance(orientation, imu_covariance_multiplier_, covariance_floor_);
    condition_covariance(angular, imu_covariance_multiplier_, covariance_floor_);
    condition_covariance(acceleration, imu_covariance_multiplier_, covariance_floor_);
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
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        const double gravity_term = gravity_ * gravity_ * gravity_cov[3 * row + col];
        event.covariance[(6 + row) + 16 * (6 + col)] =
          acceleration[3 * row + col] + gravity_term;
        double cross_term = 0.0;
        for (int k = 0; k < 3; ++k) {
          cross_term -= gravity_ * j[3 * row + k] * gravity_cov[3 * k + col];
        }
        event.covariance[row + 16 * (6 + col)] = cross_term;
        event.covariance[(6 + col) + 16 * row] = cross_term;
      }
    }
    enqueue(std::move(event));
  }

  void fog_callback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
  {
    const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
    auto transform = sensor_transform(msg->header.frame_id, stamp);
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
    auto transform = sensor_transform(msg->header.frame_id, stamp);
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
    std::fill(std::begin(core_->rtU.Q), std::end(core_->rtU.Q), 0.0);
    const double scale = std::max(dt, 0.0);
    for (int i = 0; i < 16; ++i) {
      if (i < 3 || i > 6) {
        core_->rtU.Q[i + 16 * i] = process_noise_diag_[i] * scale;
      }
    }
    State normalized = state_;
    normalize_quaternion(normalized);
    const double w = normalized[3], x = normalized[4];
    const double y = normalized[5], z = normalized[6];
    const double g[12] = {-x, -y, -z, w, -z, y, z, w, -x, -y, x, w};
    for (int row = 0; row < 4; ++row) {
      for (int col = 0; col < 4; ++col) {
        for (int axis = 0; axis < 3; ++axis) {
          core_->rtU.Q[(3 + row) + 16 * (3 + col)] += 0.25 * scale *
            g[row * 3 + axis] * process_noise_diag_[3 + axis] * g[col * 3 + axis];
        }
      }
    }
  }

  void capture_output()
  {
    std::copy(std::begin(core_->rtY.state), std::end(core_->rtY.state), state_.begin());
    std::copy(
      std::begin(core_->rtY.covariance), std::end(core_->rtY.covariance),
      covariance_.begin());
    const State previous = published_state_;
    normalize_state_covariance(state_, covariance_, has_published_ ? &previous : nullptr);
  }

  void predict_to(const rclcpp::Time & stamp)
  {
    double remaining = (stamp - filter_time_).seconds();
    while (remaining > 1e-9) {
      const double dt = std::min(remaining, max_prediction_dt_);
      clear_inputs();
      set_process_noise(dt);
      core_->rtU.dt = dt;
      core_->step();
      filter_time_ = filter_time_ + rclcpp::Duration::from_seconds(dt);
      remaining = (stamp - filter_time_).seconds();
    }
  }

  void process_measurement(const Measurement & event)
  {
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
      return;
    }
    if (event.stamp < filter_time_) {
      ++stale_rejections_;
      return;
    }
    last_processed_stamp_ns_[sensor_index] = event.stamp.nanoseconds();
    last_queue_age_ms_ = std::max(0.0, (get_clock()->now() - event.stamp).seconds() * 1000.0);
    predict_to(event.stamp);
    const State state_before = state_;
    last_innovation_norm_[static_cast<std::size_t>(event.sensor)] = innovation_norm(event);
    clear_inputs();
    set_process_noise(0.0);
    core_->rtU.dt = 0.0;
    switch (event.sensor) {
      case Sensor::Imu:
        core_->rtU.enable_imu = true;
        for (int i = 0; i < 3; ++i) {
          core_->rtU.imu_measurement[i] = 0.0;
          core_->rtU.imu_measurement[3 + i] = event.value[3 + i] * imu_config_[9 + i];
          core_->rtU.imu_measurement[6 + i] = event.value[6 + i] * imu_config_[12 + i];
          core_->rtU.imu_context[i] = event.value[i];
          core_->rtU.imu_context[3 + i] = imu_config_[3 + i];
          core_->rtU.imu_context[6 + i] = imu_config_[9 + i];
          core_->rtU.imu_context[9 + i] = imu_config_[12 + i];
        }
        copy_masked_covariance(event, 9, core_->rtU.imu_context + 3, core_->rtU.R_imu);
        break;
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
    core_->step();
    filter_time_ = event.stamp;
    capture_output();
    double correction_squared = 0.0;
    for (int i = 0; i < 16; ++i) {
      const double delta = state_[i] - state_before[i];
      correction_squared += delta * delta;
    }
    last_correction_norm_[static_cast<std::size_t>(event.sensor)] =
      std::sqrt(correction_squared);
    ++accepted_[static_cast<std::size_t>(event.sensor)];
  }

  double innovation_norm(const Measurement & event) const
  {
    std::array<double, 9> residual{};
    int size = 1;
    switch (event.sensor) {
      case Sensor::Imu: {
          const auto r = rotation(state_);
          const tf2::Vector3 measured_gravity(event.value[0], event.value[1], event.value[2]);
          const tf2::Vector3 predicted_gravity(r[6], r[7], r[8]);
          const tf2::Vector3 tilt = measured_gravity.cross(predicted_gravity);
          residual[0] = -tilt.x() * imu_config_[3];
          residual[1] = -tilt.y() * imu_config_[4];
          residual[2] = -tilt.z() * imu_config_[5];
          for (int i = 0; i < 3; ++i) {
            residual[3 + i] = (event.value[3 + i] - state_[10 + i]) * imu_config_[9 + i];
            residual[6 + i] = (event.value[6 + i] - state_[13 + i]) * imu_config_[12 + i];
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
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        const double value = pose->pose.covariance[row * 6 + col];
        if (std::isfinite(value)) {result[row + 16 * col] = value;}
      }
    }
    const double w = state[3], x = state[4], y = state[5], z = state[6];
    const double g[12] = {-x, -y, -z, w, -z, y, z, w, -x, -y, x, w};
    for (int row = 0; row < 4; ++row) {
      for (int col = 0; col < 4; ++col) {
        result[(3 + row) + 16 * (3 + col)] = 0.0;
        for (int a = 0; a < 3; ++a) {
          for (int b = 0; b < 3; ++b) {
            const double value = pose->pose.covariance[(3 + a) * 6 + 3 + b];
            if (std::isfinite(value)) {
              result[(3 + row) + 16 * (3 + col)] +=
                0.25 * g[row * 3 + a] * value * g[col * 3 + b];
            }
          }
        }
      }
    }
    return result;
  }

  void reset_filter(
    State state, const rclcpp::Time & stamp,
    const geometry_msgs::msg::PoseWithCovarianceStamped * pose = nullptr)
  {
    while (!queue_.empty()) {queue_.pop();}
    last_processed_stamp_ns_.fill(0);
    normalize_quaternion(state, has_published_ ? &published_state_ : nullptr);
    core_ = std::make_unique<talos_ekf>();
    core_->initialize();
    clear_inputs();
    const Covariance desired_covariance = reset_covariance(state, pose);
    set_process_noise(0.0);
    core_->rtU.dt = 0.0;
    core_->rtU.enable_reset = true;
    std::copy(state.begin(), state.end(), core_->rtU.reset_state);
    std::fill(std::begin(core_->rtU.R_reset), std::end(core_->rtU.R_reset), 0.0);
    for (int i = 0; i < 16; ++i) {
      core_->rtU.R_reset[i + 16 * i] = 1e-12;
    }
    core_->step();
    // The reset measurement installs the requested state. Add the requested
    // uncertainty in a zero-time prediction, then expose that internal result.
    clear_inputs();
    core_->rtU.dt = 0.0;
    std::copy(desired_covariance.begin(), desired_covariance.end(), core_->rtU.Q);
    core_->step();
    clear_inputs();
    set_process_noise(0.0);
    core_->rtU.dt = 0.0;
    core_->step();
    filter_time_ = stamp;
    capture_output();
    initialized_ = true;
    ++resets_;
    RCLCPP_INFO(
      get_logger(), "Reset EKF at %.3f s to position [%.3f, %.3f, %.3f]",
      stamp.seconds(), state_[0], state_[1], state_[2]);
  }

  std::optional<State> pose_state(const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
  {
    geometry_msgs::msg::Pose pose = msg.pose.pose;
    if (!msg.header.frame_id.empty() && msg.header.frame_id != world_frame_) {
      try {
        geometry_msgs::msg::PoseStamped source, transformed;
        source.header = msg.header; source.pose = pose;
        tf2::doTransform(
          source, transformed,
          tf_buffer_.lookupTransform(
            world_frame_, msg.header.frame_id,
            rclcpp::Time(msg.header.stamp, get_clock()->get_clock_type())));
        pose = transformed.pose;
      } catch (const tf2::TransformException &) {
        ++transform_rejections_;
        return std::nullopt;
      }
    }
    State result{};
    result[0] = pose.position.x; result[1] = pose.position.y; result[2] = pose.position.z;
    result[3] = pose.orientation.w; result[4] = pose.orientation.x;
    result[5] = pose.orientation.y; result[6] = pose.orientation.z;
    if (!normalize_quaternion(result)) {return std::nullopt;}
    return result;
  }

  void set_pose_topic(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(filter_mutex_);
    if (auto requested = pose_state(*msg)) {
      reset_filter(*requested, get_clock()->now(), msg.get());
    }
  }

  void set_pose_service(
    const std::shared_ptr<robot_localization::srv::SetPose::Request> request,
    std::shared_ptr<robot_localization::srv::SetPose::Response>)
  {
    std::lock_guard<std::mutex> lock(filter_mutex_);
    if (auto requested = pose_state(request->pose)) {
      reset_filter(*requested, get_clock()->now(), &request->pose);
    }
  }

  void fill_pose_covariance(nav_msgs::msg::Odometry & msg, const Covariance & p, const State & x)
  {
    const double w = x[3], qx = x[4], qy = x[5], qz = x[6];
    const double j[12]{-2 * qx, 2 * w, 2 * qz, -2 * qy,
      -2 * qy, -2 * qz, 2 * w, 2 * qx,
      -2 * qz, 2 * qy, -2 * qx, 2 * w};
    double mapping[96]{};
    for (int i = 0; i < 3; ++i) {mapping[i * 16 + i] = 1.0;}
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 4; ++col) {
        mapping[(3 + row) * 16 + 3 + col] = j[row * 4 + col];
      }
    }
    for (int row = 0; row < 6; ++row) {
      for (int col = 0; col < 6; ++col) {
        double value = 0.0;
        for (int a = 0; a < 16; ++a) {
          for (int b = 0; b < 16; ++b) {
            value += mapping[row * 16 + a] * p[a + 16 * b] * mapping[col * 16 + b];
          }
        }
        msg.pose.covariance[row * 6 + col] = value;
      }
    }
  }

  void publish(const rclcpp::Time & stamp)
  {
    double remaining = std::max(0.0, (stamp - filter_time_).seconds());
    State output = state_;
    Covariance output_covariance = covariance_;
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
    status.values.push_back(key_value("queue_depth", queue_.size()));
    status.values.push_back(key_value("resets", resets_));
    status.values.push_back(key_value("stale_rejections", stale_rejections_));
    status.values.push_back(key_value("transform_rejections", transform_rejections_));
    status.values.push_back(key_value("invalid_rejections", invalid_rejections_));
    status.values.push_back(key_value("queue_rejections", queue_rejections_));
    status.values.push_back(key_value("duplicate_rejections", duplicate_rejections_));
    status.values.push_back(key_value("startup_epoch_alignments", startup_epoch_alignments_));
    status.values.push_back(key_value("imu_accepted", accepted_[0]));
    status.values.push_back(key_value("fog_accepted", accepted_[1]));
    status.values.push_back(key_value("dvl_accepted", accepted_[2]));
    status.values.push_back(key_value("depth_accepted", accepted_[3]));
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

  void timer_callback()
  {
    std::lock_guard<std::mutex> lock(filter_mutex_);
    const rclcpp::Time now = get_clock()->now();
    if (filter_time_.nanoseconds() == 0 && now.nanoseconds() > 0) {
      filter_time_ = queue_.empty() ? now : queue_.top().stamp;
      ++startup_epoch_alignments_;
    }
    if (last_clock_.nanoseconds() != 0 && now < last_clock_) {
      reset_filter(state_, now);
      last_publish_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      last_diagnostics_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    }
    last_clock_ = now;
    const rclcpp::Time watermark = now - rclcpp::Duration::from_seconds(
      std::max(
        0.0,
        reorder_delay_));
    std::size_t processed = 0;
    while (!queue_.empty() && queue_.top().stamp <= watermark &&
      processed < max_measurements_per_cycle_)
    {
      Measurement event = queue_.top(); queue_.pop(); process_measurement(event);
      ++processed;
    }
    if (initialized_ && (last_publish_.nanoseconds() == 0 || now > last_publish_)) {
      publish(now); last_publish_ = now;
    }
    if (last_diagnostics_.nanoseconds() == 0 || (now - last_diagnostics_).seconds() >= 1.0) {
      publish_diagnostics(now); last_diagnostics_ = now;
    }
  }

  std::string world_frame_, base_frame_;
  bool publish_tf_{}, publish_debug_diagnostics_{};
  double frequency_{}, gravity_{}, reorder_delay_{}, max_prediction_dt_{};
  std::size_t max_queue_size_{}, max_measurements_per_cycle_{};
  std::vector<double> process_noise_diag_, reset_covariance_diag_;
  std::array<double, kConfigSize> imu_config_{}, dvl_config_{}, fog_config_{}, depth_config_{};
  double default_orientation_variance_{}, default_angular_velocity_variance_{};
  double default_acceleration_variance_{}, default_fog_variance_{}, default_dvl_variance_{},
    default_depth_variance_{};
  double imu_covariance_multiplier_{}, fog_covariance_multiplier_{}, dvl_covariance_multiplier_{};
  double depth_covariance_multiplier_{}, covariance_floor_{};
  std::unique_ptr<talos_ekf> core_;
  State state_{}, published_state_{};
  Covariance covariance_{};
  bool initialized_{false}, has_published_{false};
  rclcpp::Time filter_time_{0, 0, RCL_ROS_TIME}, last_clock_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_publish_{0, 0, RCL_ROS_TIME}, last_diagnostics_{0, 0, RCL_ROS_TIME};
  std::priority_queue<Measurement, std::vector<Measurement>, EarlierMeasurement> queue_;
  std::uint64_t sequence_{}, resets_{}, stale_rejections_{}, transform_rejections_{};
  std::uint64_t invalid_rejections_{}, queue_rejections_{}, duplicate_rejections_{};
  std::uint64_t startup_epoch_alignments_{};
  std::array<std::uint64_t, static_cast<std::size_t>(Sensor::Count)> accepted_{};
  std::array<std::int64_t, static_cast<std::size_t>(Sensor::Count)> last_processed_stamp_ns_{};
  std::array<double, static_cast<std::size_t>(Sensor::Count)> last_innovation_norm_{};
  std::array<double, static_cast<std::size_t>(Sensor::Count)> last_correction_norm_{};
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
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
