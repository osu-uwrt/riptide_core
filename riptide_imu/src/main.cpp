#include <chrono>
#include <queue>

#include <vn/sensors.h>
#include <vn/compositedata.h>
#include <vn/exceptions.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>
#include <riptide_msgs2/msg/vn_dump.hpp>
#include <riptide_msgs2/msg/vn_vpe_status.hpp>

#include <fcntl.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <riptide_msgs2/action/mag_cal.hpp>
#include <riptide_msgs2/msg/imu_config.hpp>
#include <riptide_msgs2/srv/query_imu_serial.hpp>

using namespace vn::sensors;
using namespace std::placeholders;
using namespace std::chrono_literals;

class Vectornav : public rclcpp::Node {
  using MagCal = riptide_msgs2::action::MagCal;
  using MagCalGH = rclcpp_action::ServerGoalHandle<MagCal>;
  using SerialRequest = riptide_msgs2::srv::QueryImuSerial;


  public:
  Vectornav() : Node("riptide_imu") {
    // Declare parameters used in constructor
    auto port = declare_parameter<std::string>("port", "/dev/ttyTHS0");
    auto baud = declare_parameter<int>("baud", 921600);
    auto reconnectMS = std::chrono::milliseconds(declare_parameter<int>("reconnect_ms", 500));

    // Declare parameters not used in constructor
    declare_parameter<std::string>("frame_id", "talos/imu_link");
    declare_parameter("VNErrorType", rclcpp::PARAMETER_STRING_ARRAY);

    // Data Covariance parameters
    declare_parameter("orientation_covariance", rclcpp::PARAMETER_DOUBLE_ARRAY);
    declare_parameter("angular_velocity_covariance", rclcpp::PARAMETER_DOUBLE_ARRAY);
    declare_parameter("linear_acceleration_covariance", rclcpp::PARAMETER_DOUBLE_ARRAY);

    // Magnetometer parameters
    declare_parameter<float>("magneticDeclination", 0.0f);

    // Create publisher
    imuPub = this->create_publisher<sensor_msgs::msg::Imu>("vectornav/imu", 10);
    magPub = this->create_publisher<geometry_msgs::msg::Vector3>("vectornav/magnetometer", 10);
    magHeadingPub = this->create_publisher<std_msgs::msg::Float32>("vectornav/magheading", 10);
    pressurePub = this->create_publisher<std_msgs::msg::Float32>("vectornav/pressure_bar", 10);
    dumpPub = this->create_publisher<riptide_msgs2::msg::VnDump>("vectornav/dump", 10);

    // Mag cal server
    magCalServer = rclcpp_action::create_server<MagCal>(
      this, "vectornav/mag_cal",
      std::bind(&Vectornav::handleCalGoal, this, _1, _2),
      std::bind(&Vectornav::handleCalCancel, this, _1),
      std::bind(&Vectornav::handleCalAccept, this, _1)
    );

    configSrv = this->create_service<SerialRequest>("vectornav/config", std::bind(&Vectornav::configSrvRequest, this, _1, _2));

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
    // This operation will fail if the port isn't present
    try {
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
      RCLCPP_INFO(get_logger(), "Set IMU port to ASYNC_LOW_LATENCY");
    } catch (...) {}
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
    bool imuConnected = false;
    for(auto b: baudrates) {
      try {
        vs->connect(port, b);
        if(vs->verifySensorConnectivity()) {
          // VN successfully connected
          imuConnected = true;
          break;
        }
        // Connection failed; disconnect so we can try again with a different baud
        vs->disconnect();
      } catch(...) {
        // Just catch everything
        // Doesn't matter what you do with it, loop continues
      }
    }

    // If all baudrates failed, throw an error and return
    if(!imuConnected) {
      RCLCPP_ERROR(get_logger(), "Unable to connect to IMU over port %s", port.c_str());
      vs.reset();
      return false;
    }

    // Tare
    vs->reset();

    // Wait one second and make sure VN is still connected, just to be safe
    rclcpp::Rate threadSleep(1.0);
    threadSleep.sleep();

    if(!vs->verifySensorConnectivity()) {
      RCLCPP_ERROR(get_logger(), "Lost IMU connection via %s", port.c_str());
      return false;
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
    auto magMsg = geometry_msgs::msg::Vector3();
    auto headingMsg = std_msgs::msg::Float32();
    auto pressureMsg = std_msgs::msg::Float32();
    auto dumpMsg = riptide_msgs2::msg::VnDump();
    auto vpeMsg = riptide_msgs2::msg::VnVpeStatus();

    try {
      // Parse into compositedata
      CompositeData cd = cd.parse(asyncPacket);

      // Parse message data
      msg.header.stamp = node->getTimeStamp();
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
      msg.linear_acceleration.x = acceleration.x;
      msg.linear_acceleration.y = acceleration.y;
      msg.linear_acceleration.z = acceleration.z;

      // Fill covariance data
      node->fillCovarianceFromParam("orientation_covariance", msg.orientation_covariance);
      node->fillCovarianceFromParam("angular_velocity_covariance", msg.angular_velocity_covariance);
      node->fillCovarianceFromParam("linear_acceleration_covariance", msg.linear_acceleration_covariance);

      pressureMsg.data = cd.pressure() / 100.0f;

      // Dump diagnostic topic
      dumpMsg.time_startup = cd.timeStartup();
      dumpMsg.uncomp_mag = toMsg(cd.magneticUncompensated());
      dumpMsg.uncomp_accel = toMsg(cd.accelerationUncompensated());
      dumpMsg.uncomp_gyro = toMsg(cd.angularRateUncompensated());
      dumpMsg.temperature = cd.temperature();
      dumpMsg.pressure = cd.pressure();
      dumpMsg.magnetic = toMsg(cd.magnetic());
      dumpMsg.angular_rate = toMsg(cd.angularRate());
      dumpMsg.yaw_pitch_roll = toMsg(cd.yawPitchRoll());
      dumpMsg.linear_body_accel = toMsg(cd.accelerationLinearBody());

      vn::protocol::uart::VpeStatus vpe = cd.vpeStatus();
      vpeMsg.acc_disturbance = vpe.accDisturbance;
      vpeMsg.acc_saturation = vpe.accSaturation;
      vpeMsg.attitude_quality = vpe.attitudeQuality;
      vpeMsg.gyro_saturation = vpeMsg.gyro_saturation;
      vpeMsg.gyro_saturation_recovery = vpe.gyroSaturationRecovery;
      vpeMsg.known_accel_disturbance = vpe.knownAccelDisturbance;
      vpeMsg.known_mag_disturbance = vpe.knownMagDisturbance;
      vpeMsg.mag_disturbance = vpe.magDisturbance;
      vpeMsg.mag_saturation = vpe.magSaturation;
      dumpMsg.vpe_status = vpeMsg;
    
      try {
        // magMsg = toMsg(cd.magnetic());

        try {
          // headingMsg.data = node->calculateMagnetHeading(magMsg);
        } catch (...) {
          RCLCPP_WARN(node->get_logger(), "Failed to parse magnetometer heading");
        }
      } catch (...) {
        RCLCPP_WARN(node->get_logger(), "Invalid magnetometer data");
      }

      
      
    } catch (...) {
      // If at any point packet failed to parse, throw warning
      RCLCPP_WARN(node->get_logger(), "Failed to parse or fill binary IMU packet");
      return;
    }

    // Publish output, throw error if publish failed
    try {
      node->imuPub->publish(msg);
      // node->magPub->publish(magMsg);
      // node->magHeadingPub->publish(headingMsg);
      node->pressurePub->publish(pressureMsg);
      node->dumpPub->publish(dumpMsg);
    } catch(...) {
      RCLCPP_WARN(node->get_logger(), "IMU failed to publish a succssfully parsed packet");
    }
      
  }

  static void errorPacketReceivedHandler(void* nodeptr, vn::protocol::uart::Packet& errorPacket, size_t packetStartIndex) {
    // Get a handle to the VectorNav class
    auto node = reinterpret_cast<Vectornav*>(nodeptr);
    // Parse the error and cast it from enum to int
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
    // Don't try to reconnect during magcal
    // Doing so throws vn::timeout and crashes driver
    if (!doMonitorConnection) {
      RCLCPP_WARN(get_logger(), "IMU ignored reconnect request");
      return;
    }
    
    try {
      // Check if VN is connected
      if (vs && vs->verifySensorConnectivity())
        // All good
        return;
    } catch (...) {
      // This failing means the sensor isn't connected, so try reconnecting
    }

    RCLCPP_WARN(get_logger(), "IMU disconnected");

    try {
      // Try reconnecting
      if(vs && vs->isConnected())
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

  rclcpp::Time getTimeStamp() {
    // Return current ROS timestamp
    const rclcpp::Time t = now();
    return t;
  }

  // Process vn math (4) into geometry message
  static inline geometry_msgs::msg::Quaternion toMsg(const vn::math::vec4f & rhs) {
    geometry_msgs::msg::Quaternion lhs;
    lhs.x = rhs[0];
    lhs.y = rhs[1];
    lhs.z = rhs[2];
    lhs.w = rhs[3];
    return lhs;
  }

  // Process vn math (3) into geometry message
  static inline geometry_msgs::msg::Vector3 toMsg(const vn::math::vec3f& rhs) {
    geometry_msgs::msg::Vector3 lhs;
    lhs.x = rhs[0];
    lhs.y = rhs[1];
    lhs.z = rhs[2];
    return lhs;
  }

  // Get covariance from yaml and return to caller by reference
  void fillCovarianceFromParam(std::string paramName, std::array<double, 9>& arr) const {
    // Get copy of covariance array from param
    std::vector<double> covarianceData;
    get_parameter(paramName, covarianceData);
    
    // Copy into reference
    std::copy(covarianceData.begin(), covarianceData.end(), arr.begin());
  }

  float calculateMagnetHeading(const geometry_msgs::msg::Vector3& magnetData) {
    // Get heading
    float heading = atan2(magnetData.y, magnetData.x);

    // Offset so angle is measured between positive y-axis and magnetic North
    //heading += M_PI / 2;

    // Crush between [-Pi, Pi]
    while (heading > M_PI)
      heading -= 2 * M_PI;
    while (heading <= -M_PI)
      heading += 2 * M_PI;

    // Get magnetic declination from param
    float declination;
    get_parameter("magneticDeclination", declination);

    // Adjust heading by declination (converted to rad)
    return heading * (180 / M_PI) + (declination );
  }

  //
  // Magcal code starts here
  //

  // Handle start magcal request
  rclcpp_action::GoalResponse handleCalGoal(const rclcpp_action::GoalUUID& uuid, 
                                            std::shared_ptr<const MagCal::Goal> goal) {
    // Make sure there isn't an ongoing calibration
    if(std::thread::id() == magCalThread.get_id() && vs->verifySensorConnectivity()) {
      // All good, magcal starting
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    
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
    // Let other async threads know that magcal is happening
    doMonitorConnection = false;

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

    // Calibration begins here
    RCLCPP_WARN(get_logger(), "IMU Magnetic calibration sampling starting");

    // Deque is performant when modifying elements at start and end of list
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
      try {
        vs->writeSettings();
      } catch(...) {
        RCLCPP_ERROR(get_logger(), "IMU caught vn::timeout while writing settings");
      }
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

    // Reconnect to IMU and let async threads know magcal is done
    vnConnect(get_parameter("port").as_string(), get_parameter("baud").as_int());
    doMonitorConnection = true;
  }

    std::string executeConfigRequest(std::string request) {
        if(!vs->isConnected())
            return "IMU not connected";

        return vs->transaction(request);
    }

  void configSrvRequest(std::shared_ptr<SerialRequest::Request> request, std::shared_ptr<SerialRequest::Response> response) {
      std::string responseStr;
      if (request->request == "$VNWNV") {
          try {
              responseStr = executeConfigRequest(request->request);
          }
          catch (vn::timeout) {
            doMonitorConnection = false;
            RCLCPP_INFO(this->get_logger(), "Caught vn::timeout");
            responseStr = "$VNWNV";

            rclcpp::sleep_for(5s);
            doMonitorConnection = true;
          }
      }
      else {
          responseStr = executeConfigRequest(request->request);
      }
      
      response->response = responseStr;
  }
  

  //
  // Member variables
  //
  std::shared_ptr<VnSensor> vs;

  rclcpp::TimerBase::SharedPtr reconnectTimer;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPub;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr magPub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr magHeadingPub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pressurePub;
  rclcpp::Publisher<riptide_msgs2::msg::VnDump>::SharedPtr dumpPub;

  rclcpp_action::Server<MagCal>::SharedPtr magCalServer;
  std::thread magCalThread;

  bool doMonitorConnection = true;

  rclcpp::Service<SerialRequest>::SharedPtr configSrv;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Vectornav>());
  rclcpp::shutdown();
  return 0;
}
