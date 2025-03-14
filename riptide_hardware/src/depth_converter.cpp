#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "riptide_msgs2/msg/depth.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nortek_dvl_msgs/msg/dvl.hpp"
#include "Eigen/Dense"
#include <chrono>

using std::placeholders::_1;

using namespace std::chrono_literals;

class DepthConverter : public rclcpp::Node {
public:
    DepthConverter() : Node("depth_converter") {
        auto qos = rclcpp::SensorDataQoS();
        sub_ = this->create_subscription<riptide_msgs2::msg::Depth>(
            "state/depth/raw", qos, std::bind(&DepthConverter::depth_callback, this, _1));
        sub_dvl = this->create_subscription<nortek_dvl_msgs::msg::Dvl>(
            "dvl", qos, std::bind(&DepthConverter::depth_callback_dvl, this, _1));
        pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "depth/pose", qos);
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        std::string full_namespace = this->get_namespace();
        namespace_ = full_namespace.substr(1); // Skip the leading slash

        this->declare_parameter("use_dvl", true);
        //calculate the depth depth factor based on the dvl providing the pressure in decibar
        double dvl_depth_factor_init = -1000 * 9.81 / 10000; // db per meter
        this->declare_parameter("dvl_depth_factor", dvl_depth_factor_init);
        this->declare_parameter("dvl_varaince", .01);

        this->param_refresh_timer = this->create_wall_timer(1000ms, std::bind(&DepthConverter::refresh_parameters, this));
    }

private:
    void depth_callback(const riptide_msgs2::msg::Depth::SharedPtr msg) {
        if(this->use_dvl){
            return;
        }

        try {
            auto b2o_transform = tf_buffer_->lookupTransform("odom", namespace_ + "/base_link", tf2::TimePointZero);
            Eigen::Quaterniond b2o_orientation(
                b2o_transform.transform.rotation.w,
                b2o_transform.transform.rotation.x,
                b2o_transform.transform.rotation.y,
                b2o_transform.transform.rotation.z);
            Eigen::Matrix3d b2o_matrix = b2o_orientation.toRotationMatrix();

            if (!b2p_vector_.has_value()) {
                auto pressure_transform = tf_buffer_->lookupTransform(namespace_ + "/pressure_link", namespace_ + "/base_link", tf2::TimePointZero);
                b2p_vector_ = Eigen::Vector3d(
                    pressure_transform.transform.translation.x,
                    pressure_transform.transform.translation.y,
                    pressure_transform.transform.translation.z);
            }

            double added_depth = b2o_matrix.col(2).dot(b2p_vector_.value());

            auto out_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
            out_msg.header = msg->header;
            out_msg.header.frame_id = "odom";
            out_msg.pose.pose.position.z = msg->depth + added_depth;
            out_msg.pose.covariance[14] = msg->variance;
            out_msg.header.stamp = this->get_clock()->now();
            pub_->publish(out_msg);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "TF2 error: %s", ex.what());
        }
    }

    void depth_callback_dvl(const nortek_dvl_msgs::msg::Dvl::SharedPtr msg){
        if(!this->use_dvl){
            return;
        }

        try {
            auto b2o_transform = tf_buffer_->lookupTransform("odom", namespace_ + "/base_link", tf2::TimePointZero);
            Eigen::Quaterniond b2o_orientation(
                b2o_transform.transform.rotation.w,
                b2o_transform.transform.rotation.x,
                b2o_transform.transform.rotation.y,
                b2o_transform.transform.rotation.z);
            Eigen::Matrix3d b2o_matrix = b2o_orientation.toRotationMatrix();

            if (!b2p_vector_.has_value()) {
                auto pressure_transform = tf_buffer_->lookupTransform(namespace_ + "/dvl_link", namespace_ + "/base_link", tf2::TimePointZero);
                b2p_vector_ = Eigen::Vector3d(
                    pressure_transform.transform.translation.x,
                    pressure_transform.transform.translation.y,
                    pressure_transform.transform.translation.z);
            }

            double added_depth = b2o_matrix.col(2).dot(b2p_vector_.value());

            auto out_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
            out_msg.header = msg->header;
            out_msg.header.frame_id = "odom";
            out_msg.pose.pose.position.z = msg->pressure / dvl_depth_factor + added_depth;
            out_msg.pose.covariance[14] = dvl_variance;
            out_msg.header.stamp = this->get_clock()->now();
            pub_->publish(out_msg);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "TF2 error: %s", ex.what());
        }
    }
    void refresh_parameters(){
        this->use_dvl = this->get_parameter("use_dvl").as_bool();
        this->dvl_depth_factor = this->get_parameter("dvl_depth_factor").as_double();
        this->dvl_variance = this->get_parameter("dvl_varaince").as_double();

    }

    rclcpp::Subscription<riptide_msgs2::msg::Depth>::SharedPtr sub_;
    rclcpp::Subscription<nortek_dvl_msgs::msg::Dvl>::SharedPtr sub_dvl;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::optional<Eigen::Vector3d> b2p_vector_;
    std::string namespace_;

    rclcpp::TimerBase::SharedPtr param_refresh_timer;

    bool use_dvl = false;
    double dvl_depth_factor = 0;
    double dvl_variance = 0;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DepthConverter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}