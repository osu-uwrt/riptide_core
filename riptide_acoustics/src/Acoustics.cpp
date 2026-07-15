#include <memory>
#include <vector>
#include <algorithm>
#include <numeric>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#define MODE_MAX_FALLBACK_DIFF 1000.0f
#define P_VALUE 95.0f

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
      this->declare_parameter("mode", "avg");

      ampSubscription = this->create_subscription<std_msgs::msg::Float32>(
        "/talos/ivc/pinger/selected_freq_amp_stream", 10, std::bind(&Acoustics::ampCallback, this, _1));

      resultPublisher = this->create_publisher<std_msgs::msg::Bool>(
        "/talos/acoustics/buffer_0_closer", 10);

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
      haveResult = false;

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
        response->message = "With mode [" + this->get_parameter("mode").as_string() + "] Buffer 0 closed with " + std::to_string(buffers[0].size()) + " samples";
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

      reduceBuffers(buffers[0], buffers[1], this->get_parameter("mode").as_string());
      haveResult = true;
      publishResult();

      response->success = true;
      response->message = "mode[" + this->get_parameter("mode").as_string() + "]buffer " + std::to_string(buffer0Closer ? 0 : 1) +
                          " closer (buffer 0: " + std::to_string(result_0) +
                          ", buffer 1: " + std::to_string(result_1) + ")";
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

    void compare_avg_buffers(const std::vector<float> &buffer0, const std::vector<float> &buffer1) {
      result_0 = std::accumulate(buffer0.begin(), buffer0.end(), 0.0) / buffer0.size();
      result_1 = std::accumulate(buffer1.begin(), buffer1.end(), 0.0) / buffer1.size();
      buffer0Closer = result_0 >= result_1;
    }

    void compare_p95_buffers(const std::vector<float> &buffer0, const std::vector<float> &buffer1) {
      // it's c++ so sorting is O(n) right? (fire bar - balke)
      std::vector<float> sorted0 = buffer0;
      std::vector<float> sorted1 = buffer1;
      size_t idx_0 = static_cast<size_t>(std::clamp((double)P_VALUE, 0.0, 100.0) / 100.0 * (sorted0.size() - 1));
      size_t idx_1 = static_cast<size_t>(std::clamp((double)P_VALUE, 0.0, 100.0) / 100.0 * (sorted1.size() - 1));
      std::nth_element(sorted0.begin(), sorted0.begin() + idx_0, sorted0.end());
      std::nth_element(sorted1.begin(), sorted1.begin() + idx_1, sorted1.end());
      result_0 = sorted0[idx_0];
      result_1 = sorted1[idx_1];
      buffer0Closer = result_0 >= result_1;
    }

    void compare_max_buffers(const std::vector<float> &buffer0, const std::vector<float> &buffer1) {
      result_0 = *std::max_element(buffer0.begin(), buffer0.end());
      result_1 = *std::max_element(buffer1.begin(), buffer1.end());
      if (std::abs(result_0 - result_1) < MODE_MAX_FALLBACK_DIFF) {
        compare_avg_buffers(buffer0, buffer1);
      }
    }

    void reduceBuffers(const std::vector<float> &buffer0, const std::vector<float> &buffer1, std::string mode) {
      if (mode == "avg") {
        compare_avg_buffers(buffer0, buffer1);
      }

      if (mode == "max") {
        result_0 = *std::max_element(buffer0.begin(), buffer0.end());
        result_1 = *std::max_element(buffer1.begin(), buffer1.end());
        if (std::abs(result_0 - result_1) < MODE_MAX_FALLBACK_DIFF) {
          compare_avg_buffers(buffer0, buffer1);
        }
      }

      if (mode == "p95") {
        compare_p95_buffers(buffer0, buffer1);
      }
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
    // the results of the comparison run on the buffers
    float result_0 = 0.0f;
    float result_1 = 0.0f;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Acoustics>());
  rclcpp::shutdown();
  return 0;
}
