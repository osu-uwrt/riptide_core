#pragma once

#include <Eigen/Core>
#include <deque>
#include <optional>

namespace riptide_imu {

// Attitude drift while the sensor sits still. The rate is the least-squares slope of roll, pitch and (unwrapped)
// yaw over the trailing window; any motion restarts it, since a real turn is not drift. A mark records the
// attitude so the total change since then can be read too.
class DriftMonitor {
 public:
  explicit DriftMonitor(double window_s = 60.0, double motion_limit_rad_s = 0.05, double minimum_still_s = 10.0);
  void add(double time_s, const Eigen::Vector3d& euler_deg, const Eigen::Vector3d& gyro_rad_s);
  void reset();
  void mark();

  bool hasData() const { return has_last_; }
  bool moving() const { return moving_; }
  // How long the sensor has been still, up to the window.
  double stillSeconds() const;
  double windowSeconds() const { return window_s_; }
  double minimumStillSeconds() const { return minimum_still_s_; }
  // Roll, pitch, yaw drift in degrees per minute, once the sensor has been still for the minimum time.
  std::optional<Eigen::Vector3d> rateDegPerMinute() const;
  // Attitude change since the mark (unwrapped degrees), the time since it, and whether it moved in between.
  std::optional<Eigen::Vector3d> sinceMark() const;
  double markSeconds() const;
  bool movedSinceMark() const { return moved_since_mark_; }

 private:
  struct Entry {
    double time_s;
    Eigen::Vector3d angles;
  };
  double window_s_, motion_limit_, minimum_still_s_;
  std::deque<Entry> still_;
  bool has_last_{false};
  bool moving_{false};
  Eigen::Vector3d last_raw_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d unwrapped_{Eigen::Vector3d::Zero()};
  double last_time_{0.0};
  std::optional<Entry> mark_;
  bool moved_since_mark_{false};
};

}  // namespace riptide_imu
