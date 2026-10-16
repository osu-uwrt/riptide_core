// Copyright 2026 OSU Underwater Robotics Team
// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include "riptide_navigation/estimator_support.hpp"

namespace riptide_navigation
{

// Fixed-axis angular errors in the pose's world/header frame (left quaternion
// perturbations). Twist and IMU covariance continue to use their body frames.
inline std::array<double, 12> world_quaternion_tangent(const State & state)
{
  const double w = state[3], x = state[4], y = state[5], z = state[6];
  return {-x, -y, -z, w, z, -y, -z, w, x, y, -x, w};
}

inline std::array<double, 36> ros_pose_covariance(const State & state, const Covariance & p)
{
  const auto g = world_quaternion_tangent(state);
  std::array<double, 36> result{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      result[6 * i + j] = p[i + 16 * j];
      for (int k = 0; k < 4; ++k) {
        result[6 * i + 3 + j] += 2 * p[i + 16 * (3 + k)] * g[3 * k + j];
        result[6 * (3 + j) + i] += 2 * g[3 * k + j] * p[3 + k + 16 * i];
        for (int l = 0; l < 4; ++l) {
          result[6 * (3 + i) + 3 + j] +=
            4 * g[3 * k + i] * p[3 + k + 16 * (3 + l)] * g[3 * l + j];
        }
      }
    }
  }
  return result;
}

inline Covariance apply_ros_pose_covariance(
  const State & state, const std::array<double, 36> & pose, Covariance result)
{
  const auto g = world_quaternion_tangent(state);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      result[i + 16 * j] = pose[6 * i + j];
    }
    for (int j = 0; j < 4; ++j) {
      result[i + 16 * (3 + j)] = result[3 + j + 16 * i] = 0;
      for (int k = 0; k < 3; ++k) {
        result[i + 16 * (3 + j)] += .5 * pose[6 * i + 3 + k] * g[3 * j + k];
        result[3 + j + 16 * i] += .5 * g[3 * j + k] * pose[6 * (3 + k) + i];
      }
    }
  }
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      result[3 + i + 16 * (3 + j)] = 0;
      for (int k = 0; k < 3; ++k) {
        for (int l = 0; l < 3; ++l) {
          result[3 + i + 16 * (3 + j)] +=
            .25 * g[3 * i + k] * pose[6 * (3 + k) + 3 + l] * g[3 * j + l];
        }
      }
    }
  }
  return result;
}

}  // namespace riptide_navigation
