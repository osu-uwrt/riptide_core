#include <chrono>

#include "vn/sensors.h"
#include "vn/compositedata.h"
#include "vn/exceptions.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <fcntl.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace vn::sensors;
using namespace std::placeholders;
using namespace std::chrono_literals;

class Vectornav : public rclcpp::Node {
  public:
  Vectornav() : Node("riptide_imu") {
    // Declare parameters used in constructor
    auto port = declare_parameter<std::string>("port", "/dev/ttyUSB0");
    auto baud = declare_parameter<int>("baud", 115200);
    auto reconnectMS = std::chrono::milliseconds(declare_parameter<int>("reconnect_ms", 500));

    // Declare parameters not used in constructor
    declare_parameter<std::string>("frame_id", "vectornav");
    declare_parameter("VNErrorType", rclcpp::PARAMETER_STRING_ARRAY);

    // Data Covariance parameters
    declare_parameter("orientation_covariance", rclcpp::PARAMETER_DOUBLE_ARRAY);
    declare_parameter("angular_velocity_covariance", rclcpp::PARAMETER_DOUBLE_ARRAY);
    declare_parameter("linear_acceleration_covariance", rclcpp::PARAMETER_DOUBLE_ARRAY);

    // Create publisher
    imuPub = this->create_publisher<sensor_msgs::msg::Imu>("vectornav/imu", 10);

    optimizeSerialConnection(port);

    vnConnect(port, baud);

    // Setup spin to monitor connection (since packets are async)
    reconnectTimer = create_wall_timer(reconnectMS, std::bind(&Vectornav::monitorConnection, this));
  }

  // Clean up nicely once the node quits
  ~Vectornav() {
    // If the reconnectTimer was declared, remove it
    if(reconnectTimer) {
      reconnectTimer->cancel();
      reconnectTimer.reset();
    }

    // If VN was connected, unregister packet handlers and disconnect
    if(vs) {
      vs->unregisterErrorPacketReceivedHandler();
      vs->unregisterAsyncPacketReceivedHandler();

      if(vs->isConnected())
        vs->disconnect();
      vs.reset();
    }
  }

  private:
  void optimizeSerialConnection(const std::string& port) {
    // Assumes linux OS
    const int portFd = open(port.c_str(), O_RDWR | O_NOCTTY);

    if(portFd == -1) {
      RCLCPP_WARN(get_logger(), "Can't open imu port for optimization");
      return;
    }

    struct serial_struct serial;
    ioctl(portFd, TIOCGSERIAL, &serial);
    serial.flags |= ASYNC_LOW_LATENCY;
    ioctl(portFd, TIOCSSERIAL, &serial);
    close(portFd);
    RCLCPP_INFO(get_logger(), "Set port to ASYNC_LOW_LATENCY");
  }

  bool vnConnect(const std::string& port, const int baud) {
    // Make sure there isn't an existing instance of the sensor, 
    // then make a new one
    if(vs)
      vs.reset();
    vs = std::make_shared<VnSensor>();

    // Binary packet data callback
    vs->registerAsyncPacketReceivedHandler(this, Vectornav::asyncPacketReceivedHandler);

    // Error packet data callback
    vs->registerErrorPacketReceivedHandler(this, Vectornav::errorPacketReceivedHandler);

    // Get a list of supported baudrates and ensure selected rate is supported
    auto baudrates = vs->supportedBaudrates();
    if(std::find(baudrates.begin(), baudrates.end(), baud) == baudrates.end()) {
      RCLCPP_FATAL(get_logger(), "IMU baudrate not supported: %d", baud);
      return false;
    }

    // Try to connect with the given baudrate but retry all supported
    baudrates.insert(baudrates.begin(), baud);
    for(auto b: baudrates) {
      try {
        vs->connect(port, b);
        if(vs->verifySensorConnectivity())
          // VN successfully connected
          break;
        // Connection failed but still passed connectivity check; stop communication
        vs->disconnect();
      } catch(...) {
        // Just catch everything
        // Doesn't matter what you do with it, loop continues
      }
    }

    // If all baudrates failed, throw a fatal
    if(!vs->verifySensorConnectivity()) {
      RCLCPP_FATAL(get_logger(), "Unable to connect to IMU over port %s", port.c_str());
      return false;
    }

    // Tare
    vs->reset();

    // Wait one second and make sure VN is still connected, just to be safe
    rclcpp::Rate threadSleep(1.0);
    threadSleep.sleep();

    if(!vs->verifySensorConnectivity()) {
      RCLCPP_ERROR(get_logger(), "Lost IMU connection via %s", port.c_str());
    }

    // Query the sensor to be ABSOLUTELY SURE it's working
    std::string mn = vs->readModelNumber();
    
    RCLCPP_INFO(get_logger(), "Connected to IMU %s at baud %d over %s", mn.c_str(), vs->baudrate(), vs->port().c_str());

    return true;
  }


  static void asyncPacketReceivedHandler(
    void* nodeptr, vn::protocol::uart::Packet& asyncPacket, size_t packetStartIndex) {
    // Get a handle to the VectorNav class
    auto node = reinterpret_cast<Vectornav*>(nodeptr);
    
    // Make sure it's a binary output. If not, complain and return
    if(asyncPacket.type() != vn::protocol::uart::Packet::TYPE_BINARY) {
      RCLCPP_WARN(node->get_logger(), "IMU received incorrect packet format");
      return;
    }

    // Forward define msg so it can be used inside and outside try catch block
    auto msg = sensor_msgs::msg::Imu();

    try {
      // Parse into compositedata
      CompositeData cd = cd.parse(asyncPacket);

      // Parse message data
      msg.header.stamp = node->getTimeStamp(/*cd*/);
      msg.header.frame_id = node->get_parameter("frame_id").as_string();

      // Set quaternion data
      tf2::Quaternion q, q_ned2body;
      tf2::fromMsg(toMsg(cd.quaternion()), q);
      q_ned2body.setRPY(M_PI, 0.0, M_PI/2.0);
      msg.orientation = tf2::toMsg(q_ned2body * q);

      // Set angular velocity data
      msg.angular_velocity = toMsg(cd.angularRate());

      // Set linear acceleration data
      vn::math::vec3f acceleration = cd.acceleration();
      msg.linear_acceleration.x = -acceleration.x;
      msg.linear_acceleration.y = -acceleration.y;
      msg.linear_acceleration.z = -acceleration.z;

      // Fill covariance data
      node->fillCovarianceFromParam("orientation_covariance", msg.orientation_covariance);
      node->fillCovarianceFromParam("angular_velocity_covariance", msg.angular_velocity_covariance);
      node->fillCovarianceFromParam("linear_acceleration_covariance", msg.linear_acceleration_covariance);
    } catch (...) {
      // If at any point packet failed to parse, throw warning
      RCLCPP_WARN(node->get_logger(), "Failed to parse or fill binary IMU packet");
      return;
    }

    // Publish output, throw error if publish failed
    try {
      node->imuPub->publish(msg);
    } catch(...) {
      RCLCPP_WARN(node->get_logger(), "IMU failed to publish a succssfully parsed packet");
    }
      
  }

  static void errorPacketReceivedHandler(void* nodeptr, vn::protocol::uart::Packet& errorPacket, size_t packetStartIndex) {
    auto node = reinterpret_cast<Vectornav*>(nodeptr);
    auto err = errorPacket.parseError();
    int error = static_cast<int>(err);
    
    std::string errorType;

    // Just to be sure there isn't anything weird while error handling
    try {
      // SensorError enum from vnproglib header (types.h)
      // See config.yaml
      std::vector<std::string> validErrors;
      node->get_parameter("VNErrorType", validErrors);

      // Parse the error enum received
      if(error == 255)
        errorType = "general buffer overflow";
      else if(error <= 12)
        errorType = validErrors[error - 1];
      else
        errorType = "unknown";

      RCLCPP_ERROR(node->get_logger(), "IMU encountered %s error", errorType.c_str());
    } catch(...) {
      RCLCPP_ERROR(node->get_logger(), "Unknown error handling IMU error request");
    }
  }

  void monitorConnection() {
    // Check if VN is connected
    if(vs && vs->verifySensorConnectivity())
      // All good
      return;

    RCLCPP_WARN(get_logger(), "IMU disconnected");

    try {
      // Try reconnecting
      if(vs->isConnected())
        vs->disconnect();

      std::string port = get_parameter("port").as_string();
      int baud = get_parameter("baud").as_int();

      // If the reconnect fails, raise the alarm
      if(!vnConnect(port, baud))
        RCLCPP_WARN(get_logger(), "Failed a reconnect to IMU. Retyring...");

    } catch(...) {
      // Something has gone terribly wrong. Scream and cry about it
      RCLCPP_ERROR(get_logger(), "Unrecoverable error thrown attemtping to reconnect to IMU");
    }
  }

  rclcpp::Time getTimeStamp(/*vn::sensors::CompositeData& data*/) {
    const rclcpp::Time t = now();
    // Function here for comapability for potential future modifications
    return t;  // Time not adjusted
  }

  static inline geometry_msgs::msg::Quaternion toMsg(const vn::math::vec4f & rhs) {
    geometry_msgs::msg::Quaternion lhs;
    lhs.x = rhs[0];
    lhs.y = rhs[1];
    lhs.z = rhs[2];
    lhs.w = rhs[3];
    return lhs;
  }

  static inline geometry_msgs::msg::Vector3 toMsg(const vn::math::vec3f& rhs) {
    geometry_msgs::msg::Vector3 lhs;
    lhs.x = rhs[0];
    lhs.y = rhs[1];
    lhs.z = rhs[2];
    return lhs;
  }

  void fillCovarianceFromParam(std::string paramName, std::array<double, 9>& arr) const {
    std::vector<double> covarianceData;
    get_parameter(paramName, covarianceData);
    
    std::copy(covarianceData.begin(), covarianceData.end(), arr.begin());
  }
  

  //
  // Member variables
  //
  std::shared_ptr<VnSensor> vs;

  rclcpp::TimerBase::SharedPtr reconnectTimer;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPub;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Vectornav>());
  rclcpp::shutdown();
  return 0;
}
