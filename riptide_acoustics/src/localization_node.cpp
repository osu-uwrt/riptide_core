#include "localization_node.hpp"
#include <chrono>

using namespace std::chrono_literals;

#define RETRY_DELAY_MS 200
#define DEFAULT_BASELINE 0.1
#define DEFAULT_TEMPERATURE 20.0
#define DEFAULT_DEPTH 0.0
#define DEFAULT_PRESSURE 0.0 

// Define the formula map with lambda functions for each formula
// Input params
// T = Temperature in C
// S = Salinity in PSU (parts per thousand)
// D = Depth in meters
// P = Pressure in pascals
const std::unordered_map<std::string, LocalizationNode::SpeedFormula> LocalizationNode::formulaMap = {

    // The UNESCO equation: Chen and Millero
    // T = temperature in degrees Celsius
    // S = salinity in Practical Salinity Units (parts per thousand)
    // P = pressure in bar
    // Range of validity: temperature 0 to 40 °C, salinity 0 to 40 parts per thousand, pressure 0 to 1000 bar
    {"UNESCO", [](double T, double S, double D, double P) -> double {
        // Convert Pascals to kPa
        const double P_bar = P / 100000.0;
        
        // Calculate Cw(T,P)
        const double Cw = (1402.388 + 5.03830*T - 5.81090e-2*T*T + 3.3432e-4*T*T*T 
                          - 1.47797e-6*T*T*T*T + 3.1419e-9*T*T*T*T*T) +
                         (0.153563 + 6.8999e-4*T - 8.1829e-6*T*T + 1.3632e-7*T*T*T 
                          - 6.1260e-10*T*T*T*T)*P_bar +
                         (3.1260e-5 - 1.7111e-6*T + 2.5986e-8*T*T - 2.5353e-10*T*T*T 
                          + 1.0415e-12*T*T*T*T)*P_bar*P_bar +
                         (-9.7729e-9 + 3.8513e-10*T - 2.3654e-12*T*T)*P_bar*P_bar*P_bar;

        // Calculate A(T,P)
        const double A = (1.389 - 1.262e-2*T + 7.166e-5*T*T + 2.008e-6*T*T*T - 3.21e-8*T*T*T*T) +
                        (9.4742e-5 - 1.2583e-5*T - 6.4928e-8*T*T + 1.0515e-8*T*T*T - 2.0142e-10*T*T*T*T)*P_bar +
                        (-3.9064e-7 + 9.1061e-9*T - 1.6009e-10*T*T + 7.994e-12*T*T*T)*P_bar*P_bar +
                        (1.100e-10 + 6.651e-12*T - 3.391e-13*T*T)*P_bar*P_bar*P_bar;

        // Calculate B(T,P)
        const double B = -1.922e-2 - 4.42e-5*T + (7.3637e-5 + 1.7950e-7*T)*P_bar;

        // Calculate D(T,P)
        const double D_term = 1.727e-3 - 7.9836e-6*P_bar;

        return Cw + A*S + B*pow(S, 1.5) + D_term*S*S;
    }},

    // Coppens
    // t = T/10 where T = temperature in degrees Celsius
    // S = salinity in parts per thousand
    // D = depth in kilometres
    // Range of validity: temperature 0 to 35 °C, salinity 0 to 45 parts per thousand, depth 0 to 4000 m
    {"Coppens", [](double T, double S, double D, double P) -> double {
      // Convert temperature to t = T/10
      const double t = T / 10.0;
      
      // Convert depth from meters to kilometers
      const double D_km = D / 1000.0;
      
      // Calculate c(0,S,t) first
      const double c_0_S_t = 1449.05 + 45.7 * t - 5.21 * t * t + 0.23 * t * t * t + 
                            (1.333 - 0.126 * t + 0.009 * t * t) * (S - 35.0);
      
      // Calculate the depth terms
      const double depth_term = (16.23 + 0.253 * t) * D_km + 
                              (0.213 - 0.1 * t) * D_km * D_km;
      
      // Calculate the salinity-temperature-depth interaction term
      const double interaction_term = (0.016 + 0.0002 * (S - 35.0)) * (S - 35.0) * t * D_km;
      
      // Combine all terms for final speed calculation
      return c_0_S_t + depth_term + interaction_term;
    }},

    // Mackenzie
    // T = temperature in degrees Celsius
    // S = salinity in parts per thousand
    // D = depth in metres
    // Range of validity: temperature 2 to 30 °C, salinity 25 to 40 parts per thousand, depth 0 to 8000 m
    {"Mackenzie", [](double T, double S, double D, double P) -> double {
        return 1448.96 + 4.591 * T - 0.05304 * T * T + 0.0002374 * T * T * T
               + 1.34 * (S - 35.0) + 0.0163 * D + 1.675e-7 * D * D
               - 0.01025 * T * (S - 35.0) - 7.139e-13 * T * D * D * D;
    }},

    // DelGrosso
    // T = temperature in degrees Celsius
    // S = salinity in Practical Salinity Units
    // P = pressure in kg/cm2
    // Range of validity: temperature 0 to 30 °C, salinity 30 to 40 parts per thousand, pressure 0 to 1000 kg/cm2, where 100 kPa=1.019716 kg/cm2
    {"DelGrosso", [](double T, double S, double D, double P) -> double {
        return 1402.5 + 5.03 * T - 0.0551 * T * T + 0.000221 * T * T * T
               + (1.34 - 0.01 * T) * (S - 35.0) + 0.0163 * D;
    }},

    // LubbersGraaf
    // T = temperature in degrees Celsius
    // Range of validity: Not provided
    {"LubbersGraaf", [](double T, double S, double D, double P) -> double {
        return 1405.03 + 4.624 * T - 0.0383 * T * T;
    }},

    // Wilson
    // T = temperature in degrees Celsius
    // S = salinity in parts per thousand
    // D = depth in meters
    // Range of validity: Not provided. Generally used for typical oceanic conditions.
    {"Wilson", [](double T, double S, double D, double P) -> double {
        return 1449.2 + 4.6 * T - 0.055 * T * T + 0.00029 * T * T * T
              + (1.34 - 0.010 * T) * (S - 35.0) + 0.0163 * D
              + 1.675e-7 * D * D - 0.01025 * T * (S - 35.0);
    }},

    // Marczak
    // T = temperature in degrees Celsius.
    // Range of validity: Not Provided
    {"Marczak", [](double T, double S, double D, double P) -> double {
        return 1.402385e3 + 5.038813 * T - 5.799136e-2 * T * T
              + 3.287156e-4 * T * T * T - 1.398845e-6 * T * T * T * T
              + 2.787860e-9 * T * T * T * T * T;
    }}
};

LocalizationNode::LocalizationNode()
  : Node("localization"),
    tf_buffer(this->get_clock()),
    tf_listener(tf_buffer),
    hydrophonesCached(false),
    lastDeltaT(0.0),
    lastBearing(NAN),
    diagnosticError(""),
    diagUpdater(this)
{
  // Ros params
  this->declare_parameter<double>("default_salinity", 0.0); // Defaulting to 0
  this->declare_parameter<std::string>("sound_speed_formula", "UNESCO"); // Defaulting to UNESCO formula

  // Default hydrophone baseline in meters
  defaultHydrophoneBaseline = DEFAULT_BASELINE;

  // Default sensor values
  temperature = DEFAULT_TEMPERATURE;
  depth = DEFAULT_DEPTH;
  pressure = DEFAULT_PRESSURE;
  salinity = this->get_parameter("default_salinity").as_double();

  // Subscribers
  temperature_sub = this->create_subscription<std_msgs::msg::Float32>(
      "state/depth/temperature", 10,
      std::bind(&LocalizationNode::temperature_callback, this, std::placeholders::_1));


  depth_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "depth/pose", 10,
      std::bind(&LocalizationNode::depth_callback, this, std::placeholders::_1));

  // I know we don't have it but maybe someday
  salinity_sub = this->create_subscription<std_msgs::msg::Float32>(
      "salinity", 10,
      std::bind(&LocalizationNode::salinity_callback, this, std::placeholders::_1));

  delta_t_sub = this->create_subscription<std_msgs::msg::Float32>(
      "acoustics/delta_t", 10,
      std::bind(&LocalizationNode::delta_t_callback, this, std::placeholders::_1));

  // I added this to firmware, waiting on Ethan to look it over/test it
  pressure_sub = this->create_subscription<std_msgs::msg::Float32>(
      "state/depth/pressure", 10,
      std::bind(&LocalizationNode::pressure_callback, this, std::placeholders::_1));

  // Publisher for bearing
  bearing_pub = this->create_publisher<std_msgs::msg::Float64>("localization_bearing", 10);

  // Set up the diagnostic updater (gotta flex the panel somehow)
  diagUpdater.setHardwareID("LocalizationNode");
  diagUpdater.add("Localization Diagnostics", this, &LocalizationNode::updateDiagnostics);

  // Trigger an update every second
  diag_timer = this->create_wall_timer(1s, std::bind(&LocalizationNode::diagnosticTimerCallback, this));

  // Calculate the speed of sound
  updateSpeedOfSound();

  // Lookup the hydrophone TF frames (for computing baseline)
  lookupHydrophones();
}

void LocalizationNode::updateDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  std::string formula = this->get_parameter("sound_speed_formula").as_string();

  stat.add("Sound Speed Formula", formula);
  stat.add("Speed of Sound (m/s)", speedOfSound);
  stat.add("Hydrophone Baseline (m)", cachedBaseline.length());
  stat.add("Temperature (°C)", temperature);
  stat.add("Depth (m)", depth);
  stat.add("Salinity (ppt)", salinity);
  stat.add("Last delta_t (s)", lastDeltaT);

  if (std::isnan(lastBearing)) {
    stat.add("Last Bearing (deg)", "N/A");
  } else {
    stat.add("Last Bearing (deg)", lastBearing);
  }

  if (!diagnosticError.empty()) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, diagnosticError);
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Localization Node OK");
  }
}

void LocalizationNode::diagnosticTimerCallback()
{
  diagUpdater.force_update();
}

// Compute the speed of sound using the chosen formula
double LocalizationNode::computeSpeedOfSound(double T, double S, double D, double P) {
    std::string formulaStr = this->get_parameter("sound_speed_formula").as_string();

    auto formulaIt = formulaMap.find(formulaStr);
    if (formulaIt != formulaMap.end()) {
        return formulaIt->second(T, S, D, P);
    }
    
    RCLCPP_WARN(this->get_logger(), "Unknown sound speed formula: %s", formulaStr.c_str());
    return -1;
}


// Try and compute the speed of sound in the water
void LocalizationNode::updateSpeedOfSound()
{
  speedOfSound = computeSpeedOfSound(temperature, salinity, depth, pressure);
}

// Get the hydrophone TF frames and get the baseline
void LocalizationNode::lookupHydrophones()
{
  geometry_msgs::msg::TransformStamped left_tf;
  geometry_msgs::msg::TransformStamped right_tf;
  const std::chrono::milliseconds RETRY_DELAY(RETRY_DELAY_MS);

  RCLCPP_INFO(this->get_logger(), "Waiting for hydrophone TF transforms...");

  while (true)
  {
    try {
      left_tf = tf_buffer.lookupTransform("base_link", "hydrophone_left", tf2::TimePointZero);
      right_tf = tf_buffer.lookupTransform("base_link", "hydrophone_right", tf2::TimePointZero);
      break; // Successfully obtained both transforms
    } catch (tf2::TransformException &e) {
      RCLCPP_DEBUG(this->get_logger(), "TF lookup failed: %s. Retrying...", e.what());
      std::this_thread::sleep_for(RETRY_DELAY);
    }
  }

  // Get the left and right hydrophone positions
  leftPos = tf2::Vector3(
      left_tf.transform.translation.x,
      left_tf.transform.translation.y,
      left_tf.transform.translation.z);
  rightPos = tf2::Vector3(
      right_tf.transform.translation.x,
      right_tf.transform.translation.y,
      right_tf.transform.translation.z);

  // Compute the baseline
  cachedBaseline = computeBaseline(leftPos, rightPos);
  double d = cachedBaseline.length();

  // If we get an invalid baseline, use the default value
  if (d < 1e-6) {
    RCLCPP_WARN(this->get_logger(), "Cached hydrophone baseline too small, using default baseline of %f.", defaultHydrophoneBaseline);
    cachedBaseline = tf2::Vector3(0.0, defaultHydrophoneBaseline, 0.0);
  }

  // Cache successful hydrophone TF lookup
  hydrophonesCached = true;
}

// Compute the baseline between the two hydrophones
tf2::Vector3 LocalizationNode::computeBaseline(const tf2::Vector3 & leftPos, const tf2::Vector3 & rightPos)
{
  tf2::Vector3 leftH(leftPos.x(), leftPos.y(), 0.0);
  tf2::Vector3 rightH(rightPos.x(), rightPos.y(), 0.0);
  return rightH - leftH;
}

tf2::Vector3 LocalizationNode::selectBroadside(const tf2::Vector3 & baseline)
{
  // Compute two possible broadside direction candidates, which are perpendicular to the baseline
  tf2::Vector3 candidate1(-baseline.y(), baseline.x(), 0.0);
  tf2::Vector3 candidate2(baseline.y(), -baseline.x(), 0.0);

  // Normalize both
  candidate1.normalize();
  candidate2.normalize();

  // Define a reference forward vector (aligned with x-axis)
  tf2::Vector3 baseForward(1.0, 0.0, 0.0);

  // Compute dot products to determine which candidate is more aligned with the x-axis
  double dot1 = candidate1.dot(baseForward);
  double dot2 = candidate2.dot(baseForward);

  // Return the candidate that is most aligned with the x-axis
  return (dot1 >= dot2) ? candidate1 : candidate2;
}


void LocalizationNode::delta_t_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
  double deltaT = static_cast<double>(msg->data);
  lastDeltaT = deltaT;

  // Make sure it's a finite number
  if (!std::isfinite(deltaT)) {
    diagnosticError = "delta_t is not finite. Ignoring message.";
    return;
  }

  // Check if deltaT is too small
  if (std::abs(deltaT) < std::numeric_limits<double>::epsilon()) {
    diagnosticError = "delta_t is zero. No time difference detected.";
    return;
  }

  // Ensure that hydrophone data is available before proceeding
  if (!hydrophonesCached) {
    diagnosticError = "Hydrophone data not cached.";
    return;
  }

  // Get the baseline distance between hydrophones
  double d = cachedBaseline.length();

  // Check if the baseline is too small
  if (d < 1e-6) {
    diagnosticError = "Cached horizontal baseline too small.";
    return;
  }

  // Compute sin(theta) based on arrival time difference
  double sineTheta = (speedOfSound * deltaT) / d;

  // Make sure sineTheta can actually go into asin (-1 to 1)
  if (std::abs(sineTheta) > 1.0) {
  diagnosticError = "Computed sineTheta " + std::to_string(sineTheta) + " invalid, delta t is too high and/or baseline is too small.";
    return;
  }

  // Compute the raw angle using arcsin on sineTheta
  double rawAngle = std::asin(sineTheta);

  // Select the correct broadside (perpendicular) direction based on the cached baseline
  tf2::Vector3 broadside = selectBroadside(cachedBaseline);

  // Rotate the broadside direction by the computed angle to correct for roll/pitch/elevation difference
  tf2::Quaternion rot;
  rot.setRPY(0.0, 0.0, rawAngle);
  tf2::Vector3 pingerDir = tf2::quatRotate(rot, broadside);
  pingerDir.normalize();

  // Convert the direction vector to an angle in in the horizontal plane (in degrees not sure if we want something else instead)
  double bearingRad = std::atan2(pingerDir.y(), pingerDir.x());
  lastBearing = bearingRad * (180.0 / M_PI);

  // Clear old errors
  diagnosticError = "";  

  // Publish the computed bearing as a ROS message
  std_msgs::msg::Float64 bearingMsg;
  bearingMsg.data = lastBearing;
  bearing_pub->publish(bearingMsg);
}

// Get the temperature of the water
void LocalizationNode::temperature_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
  temperature = static_cast<double>(msg->data);
  updateSpeedOfSound();
}

// Get the depth
void LocalizationNode::depth_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  depth = msg->pose.pose.position.z;
  updateSpeedOfSound();
}

// Get the pressure
void LocalizationNode::pressure_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
  pressure = static_cast<double>(msg->data);
  updateSpeedOfSound();
}

// Get the salinity (I know we don't have this... but someday?)
void LocalizationNode::salinity_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
  salinity = msg->data;
  updateSpeedOfSound();
}

// Spin er up
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LocalizationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
