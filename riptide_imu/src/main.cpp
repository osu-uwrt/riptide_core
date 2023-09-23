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
  Vectornav() : Node("vectornav") {
    // Declare parameters used in constructor
    auto port = declare_parameter<std::string>("port", "/dev/ttyUSB0");
    auto baud = declare_parameter<int>("baud", 115200);
    auto reconnectMS = std::chrono::milliseconds(declare_parameter<int>("reconnect_ms", 500));

    // Declare parameters not used in constructor
    declare_parameter<int>("AsyncDataOutputType", vn::protocol::uart::VNOFF);
    declare_parameter<int>("AsyncDataOutputFrequency", 20);
    declare_parameter<std::string>("frame_id", "vectornav");

    // Covariance parameters
    declare_parameter<std::vector<double>>("orientation_covariance", defaultOrientationCovariance);
    declare_parameter<std::vector<double>>(
      "angular_velocity_covariance", defaultAngularVelocityCovariance);
    declare_parameter<std::vector<double>>(
      "linear_acceleration_covariance", defaultLinearAccelerationCovariance);

    //Binary Output Register 1 parameters
    declare_parameter<int>("BO1.asyncMode", vn::protocol::uart::AsyncMode::ASYNCMODE_BOTH);
    declare_parameter<int>("BO1.rateDivisor", 40);  // 20Hz
    declare_parameter<int>("BO1.commonField", 0x7FFF);
    declare_parameter<int>("BO1.timeField", vn::protocol::uart::TimeGroup::TIMEGROUP_NONE);
    declare_parameter<int>("BO1.imuField", vn::protocol::uart::ImuGroup::IMUGROUP_NONE);
    declare_parameter<int>(
      "BO1.gpsField",
      vn::protocol::uart::GpsGroup::GPSGROUP_FIX | vn::protocol::uart::GpsGroup::GPSGROUP_POSU);
    declare_parameter<int>(
      "BO1.attitudeField", vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_NONE);
    declare_parameter<int>(
      "BO1.insField", vn::protocol::uart::InsGroup::INSGROUP_POSECEF |
                        vn::protocol::uart::InsGroup::INSGROUP_VELBODY);
    declare_parameter<int>("BO1.gps2Field", vn::protocol::uart::GpsGroup::GPSGROUP_NONE);


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

    if(portFd == -1)
      RCLCPP_WARN(get_logger(), "Can't open imu port for optimization");

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

    // TODO: add callback for error handling

    // Binary packet data callback
    vs->registerAsyncPacketReceivedHandler(this, Vectornav::asyncPacketReceivedHandler);

    // TODO: Set better response and retransmit times based on testing
    //vs->setResponseTimeoutMs(1000);  // ms
    //vs->setRetransmitDelayMs(50);    // ms

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

    return vnConfigure();
  }

  bool vnConfigure() {
    vn::sensors::BinaryOutputRegister bor = {
      (vn::protocol::uart::AsyncMode)get_parameter("BO1.asyncMode").as_int(),
      static_cast<uint16_t>(get_parameter("BO1.rateDivisor").as_int()),
      (vn::protocol::uart::CommonGroup)get_parameter("BO1.commonField").as_int(),
      (vn::protocol::uart::TimeGroup)get_parameter("BO1.timeField").as_int(),
      (vn::protocol::uart::ImuGroup)get_parameter("BO1.imuField").as_int(),
      (vn::protocol::uart::GpsGroup)get_parameter("BO1.gpsField").as_int(),
      (vn::protocol::uart::AttitudeGroup)get_parameter("BO1.attitudeField").as_int(),
      (vn::protocol::uart::InsGroup)get_parameter("BO1.insField").as_int(),
      (vn::protocol::uart::GpsGroup)get_parameter("BO1.gps2Field").as_int()
    };

    vs->writeBinaryOutput1(bor);

    // Configure data type and frequency
    auto asyncDataOutputType = static_cast<vn::protocol::uart::AsciiAsync>(get_parameter("AsyncDataOutputType").as_int());
    vs->writeAsyncDataOutputType(asyncDataOutputType);

    int asyncDataOutputFreq = get_parameter("AsyncDataOutputFrequency").as_int();
    vs->writeAsyncDataOutputFrequency(asyncDataOutputFreq);

    // Successfuly configured
    return true;
  }

  static void asyncPacketReceivedHandler(
    void* nodeptr, vn::protocol::uart::Packet& asyncPacket, size_t packetStartIndex) {
    // Get a handle to the VectorNav class
    auto node = reinterpret_cast<Vectornav*>(nodeptr);
    
    //RCLCPP_INFO(node->get_logger(), "Grabbed IMU packet!");

    // Make sure it's a binary output
    if(asyncPacket.type() == vn::protocol::uart::Packet::TYPE_BINARY) {
      // Parse into compositedata
      CompositeData cd = cd.parse(asyncPacket);

      //RCLCPP_INFO(node->get_logger(), "Packet of correct format and parsed correctly");

      // Parse message data
      auto msg = sensor_msgs::msg::Imu();

      msg.header.stamp = node->getTimeStamp(/*cd*/);
      msg.header.frame_id = node->get_parameter("frame_id").as_string();

      //RCLCPP_INFO(node->get_logger(), "Header appended");

      // Set quaternion data
      tf2::Quaternion q, q_ned2body;
      // TODO: SOMETHING IS UP HERE
      // THIS LINE SPECIFICALLY
      tf2::fromMsg(toMsg(cd.quaternion()), q);
      toMsg(cd.quaternion());
      q_ned2body.setRPY(M_PI, 0.0, M_PI/2.0);
      msg.orientation = tf2::toMsg(q_ned2body * q);

      //RCLCPP_INFO(node->get_logger(), "Quaternion data processed");

      // Set angular velocity data
      msg.angular_velocity = toMsg(cd.angularRate());

      // Set linear acceleration data
      vn::math::vec3f acceleration = cd.acceleration();
      msg.linear_acceleration.x = -acceleration.x;
      msg.linear_acceleration.y = -acceleration.y;
      msg.linear_acceleration.z = -acceleration.z;

      //RCLCPP_INFO(node->get_logger(), "Angular vel and linear accel parsed");

      // Fill covariance data
      node->fillCovarianceFromParam("orientation_covariance", msg.orientation_covariance);
      node->fillCovarianceFromParam("angular_velocity_covariance", msg.angular_velocity_covariance);
      node->fillCovarianceFromParam("linear_acceleration_covariance", msg.linear_acceleration_covariance);

      // Publish output
      try {
        node->imuPub->publish(msg);
        //RCLCPP_INFO(node->get_logger(), "IMU data published!");
      } catch(std::exception e) {
        RCLCPP_ERROR(node->get_logger(), "IMU failed to publish: %s", e.what());
      }
      
    }
    else {
      RCLCPP_WARN(node->get_logger(), "IMU received incorrect packet format");
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

    } catch(std::exception e) {
      // Something has gone terribly wrong. Scream and cry about it
      RCLCPP_ERROR(get_logger(), "Error thrown attemtping to reconnect to IMU: %s", e.what());
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
    auto covariance = get_parameter(paramName).as_double_array();

    std::copy(covariance.begin(), covariance.end(), arr.begin());
  }
  

  //
  // Member variables
  //
  std::shared_ptr<VnSensor> vs;

  rclcpp::TimerBase::SharedPtr reconnectTimer;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPub;

  // Default covariance fields
  const std::vector<double> defaultOrientationCovariance = {0.0, 0.0, 0.0, 0.0, 0.0, 
                                                            0.0, 0.0, 0.0, 0.0};
  const std::vector<double> defaultAngularVelocityCovariance = {0.0, 0.0, 0.0, 0.0, 0.0, 
                                                                0.0, 0.0, 0.0, 0.0};
  const std::vector<double> defaultLinearAccelerationCovariance = {0.0, 0.0, 0.0, 0.0, 0.0, 
                                                                   0.0, 0.0, 0.0, 0.0};                                                               
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Vectornav>());
  rclcpp::shutdown();
  return 0;
}
