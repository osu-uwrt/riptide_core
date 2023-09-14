#include <chrono>

#include "vn/sensors.h"
#include "vn/compositedata.h"
#include "vn/exceptions.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace vn::sensors;
using namespace std::placeholders;

void messageTrampoline(void* userData, vn::protocol::uart::Packet& p, size_t index);

class VNPublisher : public rclcpp::Node {
  public:
  VNPublisher(std::string port, uint32_t baud) 
  : Node("imu_publisher"),
  vnPort(port), vnBaudrate(baud) {
    //Used for ms timings
    using namespace std::chrono_literals;

    publisher = this->create_publisher<sensor_msgs::msg::Imu>("vectornav/imu", 10);
    timer = this->create_wall_timer(1000ms, std::bind(&VNPublisher::queryVN, this));
  }

  static void parseBinaryAsyncMessage(void* userData, vn::protocol::uart::Packet& p, size_t index) {
    //Make sure we're reading a binary package
    if(p.type() != vn::protocol::uart::Packet::TYPE_BINARY)
      return;

    //Parse the raw packet into composite data
    sensor_msgs::msg::Imu msgIMU;
    CompositeData cd = CompositeData::parse(p);

    //Extract pos quaternion from composite data
    vn::math::vec4f quat = cd.quaternion();

    //Update msgIMU with pos quaternion
    msgIMU.orientation.x = quat[0];
    msgIMU.orientation.y = quat[1];
    msgIMU.orientation.z = quat[2];
    msgIMU.orientation.w = quat[3];

    //Publish the final output
    //publisher->publish(msgIMU);

    //processedIMU = true;
  }

  private:
  void attemptConnection() {
    //Initialize vectornav
    try {
      vectornav.connect(vnPort, vnBaudrate);

      //void (VNPublisher::*parserPointer)(void*, vn::protocol::uart::Packet&, size_t) const = parseBinaryAsyncMessage;
      //vectornav.registerAsyncPacketReceivedHandler(NULL, std::bind(&VNPublisher::parseBinaryAsyncMessage, this, _1, _2, _3));
    }
    catch(vn::not_found) {
      //Complain
      RCLCPP_INFO(this->get_logger(), "Failed to connect to IMU");
    }
  }

  void queryVN() {
    //Try to connect to the sensor every update in case first attempt fails.
    //Once connected, stop trying
    if(!vectornav.verifySensorConnectivity()) {
      attemptConnection();
      //Sensor isn't connected, no point checking for data
      return;
    }

    //Any data come in?
    if(processedIMU) {
      //All good
      processedIMU = false;
      return;
    }
    else {
      //Scream and cry about it
      RCLCPP_INFO(this->get_logger(), "Model Number: %s", vectornav.readModelNumber());
    }
  }

  VnSensor vectornav;
  const std::string vnPort;
  const uint32_t vnBaudrate;

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher;

  //bool connectionEstablished = false;
  bool processedIMU = false;
};

std::shared_ptr<VNPublisher> vectornav = std::make_shared<VNPublisher>("/dev/ttyUSB0", 115200);
void messageTrampoline(void* userData, vn::protocol::uart::Packet& p, size_t index) {
  vectornav->parseBinaryAsyncMessage(userData, p, index);
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(vectornav);
  rclcpp::shutdown();
  return 0;
}