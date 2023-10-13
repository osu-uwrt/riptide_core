#include <chrono>
#include <queue>

#include "vn/sensors.h"
#include "vn/compositedata.h"
#include "vn/exceptions.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <fcntl.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "riptide_msgs2/action/mag_cal.hpp"
#include "riptide_msgs2/msg/imu_config.hpp"

using namespace vn::sensors;
using namespace std::placeholders;
using namespace std::chrono_literals;
using namespace std::placeholders;

class Vectornav : public rclcpp::Node {
  using MagCal = riptide_msgs2::action::MagCal;
  using MagCalGH = rclcpp_action::ServerGoalHandle<MagCal>;


  public:
  Vectornav() : Node("riptide_imu") {
    // Declare parameters used in constructor
    auto port = declare_parameter<std::string>("port", "/dev/ttyUSB0");
    auto baud = declare_parameter<int>("baud", 230400);
    auto reconnectMS = std::chrono::milliseconds(declare_parameter<int>("reconnect_ms", 500));

    // Declare parameters not used in constructor
    declare_parameter<std::string>("frame_id", "talos/imu_link");
    declare_parameter("VNErrorType", rclcpp::PARAMETER_STRING_ARRAY);

    // Declare magnetometer parameters
    declare_parameter<bool>("hsiEnable", false);
    declare_parameter<bool>("hsiOutput", false);
    declare_parameter<int>("convergenceRate", 1);

    // Data Covariance parameters
    declare_parameter("orientation_covariance", rclcpp::PARAMETER_DOUBLE_ARRAY);
    declare_parameter("angular_velocity_covariance", rclcpp::PARAMETER_DOUBLE_ARRAY);
    declare_parameter("linear_acceleration_covariance", rclcpp::PARAMETER_DOUBLE_ARRAY);

    // Create publisher
    imuPub = this->create_publisher<sensor_msgs::msg::Imu>("vectornav/imu", 10);

    // Create config publisher and subscriber
    publishImuConfig = this->create_publisher<riptide_msgs2::msg::ImuConfig>("vectornav/config/read", 10);
    readImuConfig = this->create_subscription<riptide_msgs2::msg::ImuConfig>("vectornav/config/write", 10,
                                        std::bind(&Vectornav::hsiConfigCb, this, _1));


    // Mag cal server
    magCalServer = rclcpp_action::create_server<MagCal>(
      this, "vectornav/mag_cal",
      std::bind(&Vectornav::handleCalGoal, this, _1, _2),
      std::bind(&Vectornav::handleCalCancel, this, _1),
      std::bind(&Vectornav::handleCalAccept, this, _1)
    );

    optimizeSerialConnection(port);

    vnConnect(port, baud);

    // Send out hsi config for electrical panel
    publishHsiConfig();

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

    // Set hsi mode
    setMagControl();

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

  rclcpp_action::GoalResponse handleCalGoal(const rclcpp_action::GoalUUID& uuid, 
                                            std::shared_ptr<const MagCal::Goal> goal) {
    // Make sure there isn't an ongoing calibration
    if(std::thread::id() == magCalThread.get_id() && vs->verifySensorConnectivity())
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    
    // If there is one running, or the sensor isn't connected
    RCLCPP_WARN(get_logger(), "Mag cal already in progress or sensor not connected, rejecting request");
    return rclcpp_action::GoalResponse::REJECT;
  }

  rclcpp_action::CancelResponse handleCalCancel(const std::shared_ptr<MagCalGH> goalHandle) {
    RCLCPP_INFO(get_logger(), "Recieved request to cancel mag cal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleCalAccept(const std::shared_ptr<MagCalGH> goalHandle) {
    // Make the child thread go brrr
    magCalThread = std::thread(std::bind(&Vectornav::executeMagCal, this, _1), goalHandle);
    magCalThread.detach();
  }

  void executeMagCal(const std::shared_ptr<MagCalGH> goalHandle) {

    // Setup a ros rate timer (input is hz)
    rclcpp::Rate loopRate(4.0);

    // Make the result message in case of early abort
    auto result = std::make_shared<riptide_msgs2::action::MagCal::Result>();

    // Tell the sensor to reset magcal profile
    vn::sensors::MagnetometerCalibrationControlRegister magControl = {
      vn::protocol::uart::HsiMode::HSIMODE_RESET,
      vn::protocol::uart::HsiOutput::HSIOUTPUT_NOONBOARD,
      1 // set the convergence rate (1 slow - 5 fast)
    };

    // Cannot test for this mode as it sets, then changes immediately
    vs->writeMagnetometerCalibrationControl(magControl);

    // make sure HSI mode is now set to run as reset returns to the previous state
    magControl.hsiMode = vn::protocol::uart::HsiMode::HSIMODE_RUN;
    vs->writeMagnetometerCalibrationControl(magControl);

    // test HSI Mode is back to on
    const auto hsiMode = vs->readMagnetometerCalibrationControl();
    if(hsiMode.hsiMode != vn::protocol::uart::HsiMode::HSIMODE_RUN){
      RCLCPP_ERROR_STREAM(get_logger(), "IMU HSI mode did not return to run after reset! returned mode: " << hsiMode.hsiMode);
      goalHandle->abort(result);
      return;
    }

    //
    // Calibration begins here
    //
    RCLCPP_WARN(get_logger(), "Magnetic calibration sampling starting");

    std::deque<vn::math::mat3f> cSamples;
    std::deque<vn::math::vec3f> bSamples;

    vn::math::mat3f avgMat;
    vn::math::vec3f avgVec;

    int calSamples = 0;

    // Read current HSI calibration
    auto lastComp = vs->readCalculatedMagnetometerCalibration();

    // Collect samples until converge
    while(calSamples < 1000 && !goalHandle->is_canceling()){
      // Increment the sample counter
      calSamples++;

      // Read HSI calibration
      lastComp = vs->readCalculatedMagnetometerCalibration();

      // Push the newest samples onto a stack
      // Pop off the ones at the end of the queue
      cSamples.push_back(lastComp.c);
      if(cSamples.size() > 10){
        cSamples.pop_front();
      }
      bSamples.push_back(lastComp.b);
      if(bSamples.size() > 10){
        bSamples.pop_front();
      }

      // Create a running average of the difference over 10 samples
      for(size_t i = 1; i < cSamples.size(); i++){
        auto diffMat = cSamples.at(i-1) - cSamples.at(i);
        auto diffVec = bSamples.at(i-1) - bSamples.at(i);

        if(i < 2){
          avgMat = diffMat;
          avgVec = diffVec;
        } else {
          avgMat = (avgMat + diffMat).div(2);
          avgVec = (avgVec + diffVec).div(2);
        }
      }
      // Fill feedback message
      auto feedbackMsg = std::make_shared<riptide_msgs2::action::MagCal::Feedback>();
      feedbackMsg->samples = calSamples;

      // Populate calibration vector
      for(size_t i = 0; i < 9; i++){
        feedbackMsg->curr_calib.at(i) = lastComp.c.e[i];
      }
      feedbackMsg->curr_calib.at(9) = lastComp.b.x;
      feedbackMsg->curr_calib.at(10) = lastComp.b.y;
      feedbackMsg->curr_calib.at(11) = lastComp.b.z;

      // Populate compensation convergence vector
      for(size_t i = 0; i < 9; i++){
        feedbackMsg->curr_avg_dev.at(i) = avgMat.e[i];
      }
      feedbackMsg->curr_avg_dev.at(9) = avgVec.x;
      feedbackMsg->curr_avg_dev.at(10) = avgVec.y;
      feedbackMsg->curr_avg_dev.at(11) = avgVec.z;

      // Publish feedback to gui
      goalHandle->publish_feedback(feedbackMsg);

      // If we are in the first few samples, skip this entirely
      if(calSamples > 20){
        // Check for convergence with the all_of algorithm and bail out if we are there
        if(std::all_of(feedbackMsg->curr_avg_dev.cbegin(), feedbackMsg->curr_avg_dev.cend(),
          [](float i){ return std::fabs(i) < 1e-10; })){
            RCLCPP_WARN(get_logger(), "Mag cal has converged");
            break;
          }
      }

      // Wait a bit
      loopRate.sleep();
    }

    // If we exited normally and are not cancelling
    if(!goalHandle->is_canceling()){
      // Turn HSI mode to off to stop sampling
      // Turn HSI output to enabled
      magControl.hsiMode = vn::protocol::uart::HsiMode::HSIMODE_OFF;
      magControl.hsiOutput = vn::protocol::uart::HsiOutput::HSIOUTPUT_USEONBOARD;
      vs->writeMagnetometerCalibrationControl(magControl);

      // write the settings (new config) to VNmemory
      //try {
        vs->writeSettings();
      //} catch(...) {
        //RCLCPP_ERROR(get_logger(), "IMU caught vn::timeout while writing settings");
      //}
    }

    // Setup final deviation vector
    for(size_t i = 0; i < 9; i++){
      result->avg_dev.at(i) = avgMat.e[i];
    }
    result->avg_dev.at(9) = avgVec.x;
    result->avg_dev.at(10) = avgVec.y;
    result->avg_dev.at(11) = avgVec.z;

    // Setup final calibration vector
    for(size_t i = 0; i < 9; i++){
      result->calib.at(i) = lastComp.c.e[i];
    }
    result->calib.at(9) = lastComp.b.x;
    result->calib.at(10) = lastComp.b.y;
    result->calib.at(11) = lastComp.b.z;

    // If we stopped due to a cancellation
    if(goalHandle->is_canceling()){
      // Stopping for cancellation
      goalHandle->canceled(result);
    } else {
      // We did it
      goalHandle->succeed(result);
    }
  }

  void setMagControl() {
    vn::sensors::MagnetometerCalibrationControlRegister magControl = {
      (vn::protocol::uart::HsiMode)this->get_parameter("hsiEnable").as_bool(),
      parseHsiOutput(this->get_parameter("hsiOutput").as_bool()),
      this->get_parameter("convergenceRate").as_int()
    };
    vs->writeMagnetometerCalibrationControl(magControl);
  }

  void hsiConfigCb(const riptide_msgs2::msg::ImuConfig config) {
    // Check if the electrical panel requested to send current settings
    if(config.convergence_rate == 100) {
      RCLCPP_INFO(get_logger(), "IMU recieved config retransmit request");
      // Issue the current imu hsi config to the electrical panel
      publishHsiConfig();
      return;
    }

    RCLCPP_INFO(get_logger(), "IMU recieved new HSI config");
    this->set_parameters(std::vector<rclcpp::Parameter>{
      rclcpp::Parameter("hsiEnable", config.hsi_enable),
      rclcpp::Parameter("hsiOutput", config.hsi_output),
      rclcpp::Parameter("convergenceRate", config.convergence_rate)
    });
    setMagControl();
    publishHsiConfig();
  }
    

  void publishHsiConfig() {
    auto msg = riptide_msgs2::msg::ImuConfig();
    msg.hsi_enable = this->get_parameter("hsiEnable").as_bool();
    msg.hsi_output = this->get_parameter("hsiOutput").as_bool();
    msg.convergence_rate = this->get_parameter("convergenceRate").as_int();

    publishImuConfig->publish(msg);
  }

  vn::protocol::uart::HsiOutput parseHsiOutput(bool outputState) {
    return outputState ? vn::protocol::uart::HsiOutput::HSIOUTPUT_USEONBOARD 
                       : vn::protocol::uart::HsiOutput::HSIOUTPUT_NOONBOARD;
  }
  

  //
  // Member variables
  //
  std::shared_ptr<VnSensor> vs;

  rclcpp::TimerBase::SharedPtr reconnectTimer;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPub;

  rclcpp_action::Server<MagCal>::SharedPtr magCalServer;
  std::thread magCalThread;

  rclcpp::Publisher<riptide_msgs2::msg::ImuConfig>::SharedPtr publishImuConfig;
  rclcpp::Subscription<riptide_msgs2::msg::ImuConfig>::SharedPtr readImuConfig;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Vectornav>());
  rclcpp::shutdown();
  return 0;
}
