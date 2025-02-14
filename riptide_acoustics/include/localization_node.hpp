#ifndef RIPTIDE_ACOUSTICS_LOCALIZATION_NODE_HPP
#define RIPTIDE_ACOUSTICS_LOCALIZATION_NODE_HPP

#include <string>
#include <limits>
#include <unordered_map>
#include <functional>

#include "rclcpp/rclcpp.hpp"

// Message types
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp" 
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

// TF2 includes
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Vector3.h"

// Diagnostic Updater includes
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_updater/update_functions.hpp"

class LocalizationNode : public rclcpp::Node
{
public:
    LocalizationNode();

private:
    // Type def for sound speed calculation lambda function.
    // T = Temperature in C
    // S = Salinity in PSU (parts per thousand) lol lets pretend
    // D = Depth in meters
    // P = Pressure in pascals
    using SpeedFormula = std::function<double(double T, double S, double D, double P)>;

    // Map of formula names to calculation functions
    static const std::unordered_map<std::string, SpeedFormula> formulaMap;

    // Member variables
    double temperature;
    double depth;
    double salinity;
    double pressure;
    double speedOfSound;
    double defaultHydrophoneBaseline;

    // Cached positions and baseline
    tf2::Vector3 leftPos;
    tf2::Vector3 rightPos;
    tf2::Vector3 cachedBaseline;
    bool hydrophonesCached;

    // Latest measurements
    double lastDeltaT;
    double lastBearing;  // in degrees
    std::string diagnosticError;  // nonempty if an error occurred

    // ROS2 subscribers and publisher
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr temperature_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr depth_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pressure_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr salinity_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr delta_t_sub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr bearing_pub;

    // TF2 buffer and listener
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

    // Diagnostic Updater
    diagnostic_updater::Updater diagUpdater;
    rclcpp::TimerBase::SharedPtr diag_timer;

    // Helper functions
    double computeSpeedOfSound(double T, double S, double D, double P);
    void updateSpeedOfSound();

    // TF helper functions
    void lookupHydrophones();
    tf2::Vector3 computeBaseline(const tf2::Vector3& leftPos, const tf2::Vector3& rightPos);
    tf2::Vector3 selectBroadside(const tf2::Vector3& baseline);

    // Diagnostic functions
    void updateDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
    void diagnosticTimerCallback();

    // Callbacks
    void delta_t_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void temperature_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void depth_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void pressure_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void salinity_callback(const std_msgs::msg::Float32::SharedPtr msg);
};

#endif
