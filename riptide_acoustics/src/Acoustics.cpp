#include <memory>
#include <vector>
#include <algorithm>
#include <numeric>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::placeholders;

// 1) start_sample -> collect amplitudes for buffer 0
// 2) stop_sample -> close buffer 0
// 3) start_sample -> collect ampltudes for buffer 1
// 4) stop_sample -> close buffer 1, then compare and publish once to buffer_0_closer

// Using p95 atm, but could be changed pretty easily
class Acoustics : public rclcpp::Node
{
  public:
    Acoustics()
    : Node("riptide_acoustics")
    {
      this->declare_parameter("percentile", 95.0);

      ampSubscription = this->create_subscription<std_msgs::msg::Float32>(
        "ivc/pinger/selected_amp", 10, std::bind(&Acoustics::ampCallback, this, _1));

      resultPublisher = this->create_publisher<std_msgs::msg::Bool>(
        "acoustics/buffer_0_closer", 10);

      resultTimer = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&Acoustics::publishResult, this));

      startSampleService = this->create_service<std_srvs::srv::Trigger>(
        "acoustics/start_sample", std::bind(&Acoustics::startSample, this, _1, _2));

      stopSampleService = this->create_service<std_srvs::srv::Trigger>(
        "acoustics/stop_sample", std::bind(&Acoustics::stopSample, this, _1, _2));
    }

  private:
    void ampCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
      if (sampling) {
        buffers[activeBuffer].push_back(msg->data);
      }
    }

    void startSample(const std_srvs::srv::Trigger::Request::SharedPtr, 
                           std_srvs::srv::Trigger::Response::SharedPtr response)
    {
      if (sampling) {
        response->success = false;
        response->message = "Already sampling";
        return;
      }

      buffers[activeBuffer].clear();
      sampling = true;

      response->success = true;
      response->message = "Sampling into buffer " + std::to_string(activeBuffer);
      RCLCPP_INFO(this->get_logger(), "Started sampling into buffer %d", activeBuffer);
    }

    void stopSample(const std_srvs::srv::Trigger::Request::SharedPtr, 
                          std_srvs::srv::Trigger::Response::SharedPtr response)
    {
      if (!sampling) {
        response->success = false;
        response->message = "Not sampling";
        return;
      }

      sampling = false;

      RCLCPP_INFO(this->get_logger(), "Stopped sampling buffer %d with %zu samples", 
                                      activeBuffer, buffers[activeBuffer].size());

      if (activeBuffer == 0) {
        activeBuffer = 1;
        response->success = true;
        response->message = "Buffer 0 closed with " + std::to_string(buffers[0].size()) + " samples";
        return;
      }

      // buffer 1 just closed, compare and reset
      compareBuffers(response);
      activeBuffer = 0;
    }

    void compareBuffers(std_srvs::srv::Trigger::Response::SharedPtr response)
    {
      if (buffers[0].empty() || buffers[1].empty()) {
        response->success = false;
        response->message = "Cannot compare, buffer 0 has " + std::to_string(buffers[0].size()) +
                            " samples and buffer 1 has " + std::to_string(buffers[1].size());
        RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
        return;
      }

      float value0 = reduceBuffer(buffers[0]);
      float value1 = reduceBuffer(buffers[1]);

      buffer0Closer = value0 >= value1;
      haveResult = true;
      publishResult();

      response->success = true;
      response->message = "buffer " + std::to_string(buffer0Closer ? 0 : 1) +
                          " closer (buffer 0: " + std::to_string(value0) +
                          ", buffer 1: " + std::to_string(value1) + ")";
      RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }

    void publishResult()
    {
      if (!haveResult) {
        return;
      }

      std_msgs::msg::Bool msg;
      msg.data = buffer0Closer;
      resultPublisher->publish(msg);
    }

    float reduceBuffer(const std::vector<float> & buffer)
    {
      double p = this->get_parameter("percentile").as_double();

      // it's c++ so sorting is O(n) right?
      std::vector<float> sorted = buffer;
      size_t idx = static_cast<size_t>(std::clamp(p, 0.0, 100.0) / 100.0 * (sorted.size() - 1));
      std::nth_element(sorted.begin(), sorted.begin() + idx, sorted.end());
      return sorted[idx];
    }

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr ampSubscription;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr resultPublisher;
    rclcpp::TimerBase::SharedPtr resultTimer;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr startSampleService;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stopSampleService;

    // amplitude samples for the two comparison windows
    std::vector<float> buffers[2];
    int activeBuffer = 0;
    bool sampling = false;

    // last comparison result, invalid until the first compare completes
    bool buffer0Closer = false;
    bool haveResult = false;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Acoustics>());
  rclcpp::shutdown();
  return 0;
}
