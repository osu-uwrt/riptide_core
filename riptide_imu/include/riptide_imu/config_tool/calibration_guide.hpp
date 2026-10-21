#pragma once

#include <QJsonObject>
#include <QMap>
#include <QString>
#include <algorithm>
#include <array>
#include <deque>
#include <string>
#include <vector>

#include "riptide_imu/config_tool/calibration.hpp"

namespace riptide_imu {

// The guided procedure's live checks. Nothing here decides pass or fail; the estimators in calibration.hpp do, and
// these only tell the operator when a capture is usable and which pose to try next.

enum class CalibrationStep { Gyro = 0, Accelerometer = 1, Magnetometer = 2, Gravity = 3 };
constexpr int kCalibrationStepCount = 4;
const char* calibrationStepName(CalibrationStep step);

// Captured samples and the latest evaluation of each step. Clearing a step never touches the sensor.
struct CalibrationSession {
  std::vector<Sample> gyro, accelerometer, gravity, magnetometer, magnetometer_acceleration;
  GyroResult gyro_result;
  AccelerometerResult accelerometer_result;
  PlanarMagResult magnetometer_result;
  GravityReport gravity_report;
  std::array<bool, kCalibrationStepCount> evaluated{};
  // The register value last staged from each step's result, to tell whether it is still staged or now applied.
  std::array<QString, kCalibrationStepCount> delivered;

  void clear(CalibrationStep step);
  std::size_t sampleCount(CalibrationStep step) const;
  // Re-runs the estimator on the samples in memory; false when there are none.
  bool evaluate(CalibrationStep step, const CalibrationPolicy& policy);
  const Quality& quality(CalibrationStep step) const;
};

// The machine-readable report: every nonempty capture is reevaluated first.
QJsonObject calibrationReport(CalibrationSession& session, const CalibrationPolicy& policy);

// Direction of gravity (down) in the sensor frame from a stationary specific-force reading. The VN-100 reports
// specific force, -g on Z when it rests right side up, so this is the opposite of the reading.
Eigen::Vector3d gravityDirection(const Eigen::Vector3d& specific_force);
double angleBetweenDeg(const Eigen::Vector3d& a, const Eigen::Vector3d& b);

// Rolling window over gyro and accelerometer readings that says whether the sensor is at rest.
class StillnessMonitor {
 public:
  StillnessMonitor(double window_s, double gyro_limit_rad_s, double accel_limit_m_s2);
  void add(double time_s, const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel);
  void reset();
  bool still() const;
  bool hasData() const { return samples_.size() >= 3; }
  double gyroSpread() const { return gyro_spread_; }
  double accelSpread() const { return accel_spread_; }
  double gyroLimit() const { return gyro_limit_; }
  double accelLimit() const { return accel_limit_; }
  Eigen::Vector3d meanAccel() const { return accel_mean_; }
  Eigen::Vector3d meanGyro() const { return gyro_mean_; }

 private:
  struct Entry {
    double time_s;
    Eigen::Vector3d gyro, accel;
  };
  std::deque<Entry> samples_;
  double window_s_, gyro_limit_, accel_limit_;
  double gyro_spread_{0.0}, accel_spread_{0.0};
  Eigen::Vector3d gyro_mean_{Eigen::Vector3d::Zero()}, accel_mean_{Eigen::Vector3d::Zero()};
};

// The frame the guide talks in. Display axes are X ahead (up the page), Y right, Z down, so "flat" is gravity along
// display +Z. For the sensor alone they are the sensor's own printed axes; with a robot config they are the
// robot's (forward, right, down), so poses read "front down" whatever way the sensor is mounted.
struct GuideFrame {
  Eigen::Matrix3d display_from_sensor{Eigen::Matrix3d::Identity()};
  bool robot{false};
  QString name;  // "" for the sensor, else the robot's
  // What sits ahead, right, behind and left in the display, for labels on drawings.
  std::array<QString, 4> ends{{"+X", "+Y", "−X", "−Y"}};
};
GuideFrame sensorGuideFrame();
// `sensor_to_base` as in RobotMount: sensor axes in base_link (X forward, Y left, Z up).
GuideFrame robotGuideFrame(const QString& name, const Eigen::Matrix3d& sensor_to_base);

// One accelerometer pose: gravity tilted `tilt_deg` from the frame's flat pose toward `azimuth_deg` (0 = ahead,
// 90 = right), so the named end is the low end. `direction()` and `level()` are in the sensor frame.
struct TiltTarget {
  std::string label;
  double tilt_deg{0.0};
  double azimuth_deg{0.0};
  Eigen::Matrix3d sensor_from_display{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d direction() const;
  Eigen::Vector3d level() const;
};

// Flat, then tipped toward each edge and corner. 45 degrees keeps every axis span above the policy's 2 m/s^2 even
// when a pose is held a few degrees short, for any mounting.
std::vector<TiltTarget> accelerometerPoses(double tilt_deg = 45.0, const GuideFrame& frame = sensorGuideFrame());

// Whether `gravity_in_body` is within `tolerance_deg` of the pose, and tipped no more than 5 degrees short of a
// tilted pose. A symmetric ring alone would accept every pose held ~33 degrees over, which leaves the flat axis'
// span under the policy's 2 m/s^2.
bool poseReached(const TiltTarget& target, const Eigen::Vector3d& gravity_in_body, double tolerance_deg);
constexpr double kPoseShortfallDeg = 5.0;

// Heading turned about the gravity axis, integrated from the gyro. Coverage is the span between the furthest
// headings reached either way, so it starts at zero and only a real turn grows it; jitter around the start
// heading adds nothing. Integrating rate keeps the live display honest even when hard iron is large enough to
// distort the raw magnetic heading. `gyro_bias`, measured while still, keeps a biased gyro from creeping.
class HeadingCoverage {
 public:
  static constexpr int kSectors = 36;
  void reset(const Eigen::Vector3d& gravity_direction, const Eigen::Vector3d& gyro_bias = Eigen::Vector3d::Zero());
  void add(double time_s, const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel);
  double headingDeg() const;
  double coverageDeg() const { return std::min(360.0, high_deg_ - low_deg_); }
  double tiltVariationDeg() const { return max_tilt_deg_; }
  double yawRateDegS() const { return yaw_rate_deg_s_; }
  // 10-degree sectors (0 at the start heading, clockwise) inside the span turned through.
  std::array<bool, kSectors> sectors() const;

 private:
  Eigen::Vector3d up_{Eigen::Vector3d::UnitZ()};
  Eigen::Vector3d bias_{Eigen::Vector3d::Zero()};
  double last_time_{-1.0};
  double heading_deg_{0.0};  // unwrapped
  double low_deg_{0.0}, high_deg_{0.0};
  double max_tilt_deg_{0.0};
  double yaw_rate_deg_s_{0.0};
};

// Roll and pitch in degrees of a frame (X forward, Y right, Z down) whose gravity direction is `down`, as the
// VN-100 reports attitude: roll positive with the right side down, pitch positive nose up.
Eigen::Vector2d levelRollPitchDeg(const Eigen::Vector3d& down);

// The gravity report against a robot config: how the robot sits if the config's IMU rotation is right, and the
// rotation that would make it read level. Only a correction when the robot really was level during the capture;
// yaw cannot be seen from gravity, so the correction never changes it.
struct MountCheck {
  Eigen::Vector3d base_down{-Eigen::Vector3d::UnitZ()};  // gravity in base_link
  double roll_deg{0.0};   // ROS (REP-103) base_link roll: positive right side down
  double pitch_deg{0.0};  // ROS pitch: positive nose down
  double error_deg{0.0};  // total tilt from level
  Eigen::Vector3d corrected_rpy{Eigen::Vector3d::Zero()};  // radians, URDF order
};
MountCheck checkMount(const Eigen::Vector3d& sensor_down, const Eigen::Matrix3d& sensor_to_base);

// Compensation registers (y = C(x - B)) as semicolon text: C row by row, then B.
bool parseCompensation(const QString& text, Eigen::Matrix3d* matrix, Eigen::Vector3d* bias);
QString formatCompensation(const Eigen::Matrix3d& matrix, const Eigen::Vector3d& bias);
// 84 gyro, 25 accelerometer, 23 magnetometer; -1 for the gravity report.
int compensationRegister(CalibrationStep step);

// The register value a passed step would stage: the register's present compensation with the measured residual
// folded into its bias, through the reference-frame rotation (26). The gyro also needs startup bias 43 to be zero,
// or the correction would be applied twice.
struct CompensationProposal {
  int register_id{-1};
  QString values;
  QString error;  // empty when `values` is usable
  QList<int> unread;  // registers to read first
  Eigen::Vector3d old_bias{Eigen::Vector3d::Zero()}, new_bias{Eigen::Vector3d::Zero()};
};
CompensationProposal proposeCompensation(const CalibrationSession& session, CalibrationStep step,
                                         const QMap<int, QString>& device);

// Where a step's last staged result is now: still staged, written to the sensor, or neither (discarded, replaced,
// or never staged).
enum class Delivery { None, Staged, Applied };
Delivery deliveryState(const CalibrationSession& session, CalibrationStep step, const QMap<int, QString>& device,
                       const QMap<int, QString>& staged);

}  // namespace riptide_imu
