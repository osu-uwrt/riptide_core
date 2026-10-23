#include "riptide_imu/config_tool/drift_monitor.hpp"

#include <cmath>

namespace riptide_imu {

DriftMonitor::DriftMonitor(double window_s, double motion_limit_rad_s, double minimum_still_s)
    : window_s_(window_s), motion_limit_(motion_limit_rad_s), minimum_still_s_(minimum_still_s) {}

void DriftMonitor::reset() {
  still_.clear();
  has_last_ = false;
  moving_ = false;
  mark_.reset();
  moved_since_mark_ = false;
}

void DriftMonitor::mark() {
  moved_since_mark_ = false;
  if (has_last_) mark_ = Entry{last_time_, unwrapped_}; else mark_.reset();
}

void DriftMonitor::add(double time_s, const Eigen::Vector3d& euler_deg, const Eigen::Vector3d& gyro_rad_s) {
  if (!euler_deg.allFinite() || !gyro_rad_s.allFinite()) return;
  if (has_last_ && time_s < last_time_) reset();
  if (!has_last_) {
    unwrapped_ = euler_deg;
  } else {
    // keep each angle continuous across +-180
    for (int i = 0; i < 3; ++i) unwrapped_(i) += std::remainder(euler_deg(i) - last_raw_(i), 360.0);
  }
  last_raw_ = euler_deg;
  last_time_ = time_s;
  has_last_ = true;
  if (!mark_) mark_ = Entry{time_s, unwrapped_};

  moving_ = gyro_rad_s.norm() > motion_limit_;
  if (moving_) {
    still_.clear();
    moved_since_mark_ = true;
    return;
  }
  still_.push_back({time_s, unwrapped_});
  while (still_.size() > 2 && time_s - still_.front().time_s > window_s_) still_.pop_front();
}

double DriftMonitor::stillSeconds() const {
  return still_.size() < 2 ? 0.0 : still_.back().time_s - still_.front().time_s;
}

std::optional<Eigen::Vector3d> DriftMonitor::rateDegPerMinute() const {
  if (stillSeconds() < minimum_still_s_) return std::nullopt;
  const double t0 = still_.front().time_s;
  double n = 0, st = 0, stt = 0;
  Eigen::Vector3d sy = Eigen::Vector3d::Zero(), sty = Eigen::Vector3d::Zero();
  for (const auto& entry : still_) {
    const double t = entry.time_s - t0;
    n += 1;
    st += t;
    stt += t * t;
    sy += entry.angles;
    sty += t * entry.angles;
  }
  const double denominator = n * stt - st * st;
  if (!(denominator > 0)) return std::nullopt;
  return Eigen::Vector3d((n * sty - st * sy) / denominator * 60.0);
}

std::optional<Eigen::Vector3d> DriftMonitor::sinceMark() const {
  if (!mark_ || !has_last_) return std::nullopt;
  return Eigen::Vector3d(unwrapped_ - mark_->angles);
}

double DriftMonitor::markSeconds() const { return mark_ && has_last_ ? last_time_ - mark_->time_s : 0.0; }

}  // namespace riptide_imu
