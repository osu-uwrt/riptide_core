#include <memory>
#include <chrono>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/transform.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"

#include "riptide_acoustics/AcousticsMeasurement.h"


using namespace std::chrono_literals;
using std::placeholders::_1;


class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("riptide_acoustics")
    {
      subscription_ = this->create_subscription<std_msgs::msg::Float32>(
      "acoustics/delta_t", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));

      //initialize tf buffer and tf listener
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      portToStarboardTransformTimer = this->create_wall_timer(1s, std::bind(&MinimalSubscriber::getPortToStarboardTransform, this));
    }

  private:
    void topic_callback(const std_msgs::msg::Float32::SharedPtr msg) const
    {
      geometry_msgs::msg::TransformStamped t;

      try {
        t = tf_buffer_->lookupTransform(
          "world", "talos/base_link",
          tf2::TimePointZero);
      } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform!");
        return;
      }

      RCLCPP_INFO(this->get_logger(), "X: %f", t.transform.translation.x);

    }

    void getPortToStarboardTransform()
    {
      //stamped msg
      geometry_msgs::msg::TransformStamped stamped;

      //get transform from tf buffer
      try {

        //TODO: change to port and starboard pod frames
        stamped = tf_buffer_->lookupTransform("talos/pose_sensor_link", "talos/base_link", tf2::TimePointZero);
        
        portToStarboardTransform = stamped.transform;

        RCLCPP_INFO(this->get_logger(), "Successfuly collected port to starboard transform!");
      } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could collect port to starboard transform, retrying!");
        return;
      }

      portToStarboardTransformTimer->cancel();
    }

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;

    //tf
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    //port pod to starboard pod transform
    geometry_msgs::msg::Transform portToStarboardTransform;
    rclcpp::TimerBase::SharedPtr portToStarboardTransformTimer {nullptr};

};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}