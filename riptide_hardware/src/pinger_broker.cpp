#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

#include <chrono>


using std::placeholders::_1;
using namespace std::chrono_literals;

class PingerBroker : public rclcpp::Node {
public:
    PingerBroker() : Node("pinger_broker") {
        freqSelPub = this->create_publisher<std_msgs::msg::Int32>("ivc/pinger/set_freq_khz", 10);
        freqSelSub = this->create_subscription<std_msgs::msg::Int32>("ivc/pinger/set_freq_broker_khz", 10, std::bind(&PingerBroker::freqSetCb, this, _1));

        pubTimer = this->create_wall_timer(1000ms, std::bind(&PingerBroker::pubFreq, this));
    }

private:
    void freqSetCb(const std_msgs::msg::Int32& msg) {
        freq_khz = msg.data;
    } 

    void pubFreq() {
        std_msgs::msg::Int32 msg;
        msg.data = freq_khz;
        freqSelPub->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr freqSelPub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr freqSelSub;

    rclcpp::TimerBase::SharedPtr pubTimer;

    int freq_khz = 30;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PingerBroker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}