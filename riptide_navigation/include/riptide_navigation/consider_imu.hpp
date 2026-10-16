// Copyright 2026 OSU Underwater Robotics Team
// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include "riptide_navigation/estimator_support.hpp"

namespace riptide_navigation
{

// Solve a leading SPD block of the nine-dimensional innovation covariance.
// All local measurement matrices are row-major; state P remains column-major.
inline bool innovation_factor(
  const std::array<double, 81> & s, int size,
  std::array<double, 81> & l)
{
  for (int i = 0; i < size; ++i) {
    for (int j = 0; j <= i; ++j) {
      double v = s[9 * i + j];
      for (int k = 0; k < j; ++k) {
        v -= l[9 * i + k] * l[9 * j + k];
      }
      if (i == j) {
        if (!std::isfinite(v) || v <= 0.0) {return false;}
        l[9 * i + j] = std::sqrt(v);
      } else {
        l[9 * i + j] = v / l[9 * j + j];
      }
    }
  }
  return true;
}

// The observation and analytic normalized-quaternion Jacobian match
// talos_measure_imu_configured{,_jacobian}.m. Attitude and rate use only the
// tilt/rate marginal. Translation uses the full joint observation, including
// mount-induced rate/acceleration noise correlations. Joseph covariance works
// with this deliberately constrained gain; do not use (I-KH)P alone.
inline bool correct_imu_consider_acceleration(
  State & state, Covariance & covariance, const double * measurement,
  const double * context, const double * measurement_covariance,
  bool constrain_tilt_heading = false, bool consider_acceleration = true)
{
  State normalized = state;
  if (!normalize_quaternion(normalized)) {return false;}
  const double norm = std::sqrt(
    state[3] * state[3] + state[4] * state[4] +
    state[5] * state[5] + state[6] * state[6]);
  const double w = normalized[3], x = normalized[4];
  const double y = normalized[5], z = normalized[6];
  const double magnitude = std::max(
    std::hypot(
      std::hypot(context[0], context[1]),
      context[2]), 1e-12);
  const double g[3]{2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)};
  const double m[3]{context[0] / magnitude, context[1] / magnitude, context[2] / magnitude};
  const double cross[9]{0, -m[2], m[1], m[2], 0, -m[0], -m[1], m[0], 0};
  const double raw_j[12]{-2 * y, 2 * z, -2 * w, 2 * x,
    2 * x, 2 * w, 2 * z, 2 * y, 0, -4 * x, -4 * y, 0};
  double j[12]{};
  for (int i = 0; i < 3; ++i) {
    for (int c = 0; c < 4; ++c) {
      for (int k = 0; k < 4; ++k) {
        j[4 * i + c] += raw_j[4 * i + k] *
          ((k == c ? 1.0 : 0.0) - normalized[3 + k] * normalized[3 + c]) / norm;
      }
    }
  }
  double h[144]{}, residual[9]{};
  for (int i = 0; i < 3; ++i) {
    double tilt = 0.0;
    for (int k = 0; k < 3; ++k) {
      tilt += cross[3 * i + k] * g[k];
    }
    residual[i] = measurement[i] - context[3 + i] * tilt;
    residual[3 + i] = measurement[3 + i] - context[6 + i] * state[10 + i];
    residual[6 + i] = measurement[6 + i] -
      context[9 + i] * (state[13 + i] + magnitude * g[i]);
    h[(3 + i) * 16 + 10 + i] = context[6 + i];
    h[(6 + i) * 16 + 13 + i] = context[9 + i];
    for (int c = 0; c < 4; ++c) {
      for (int k = 0; k < 3; ++k) {
        h[i * 16 + 3 + c] += context[3 + i] * cross[3 * i + k] * j[4 * k + c];
      }
      h[(6 + i) * 16 + 3 + c] = context[9 + i] * magnitude * j[4 * i + c];
    }
  }
  double ph[144]{};
  std::array<double, 81> s{}, l{};
  // Tilt observes only the quaternion; each rate observes one state, and each
  // acceleration observes the quaternion plus one acceleration state. Keep the
  // complete correlated P/R and Joseph update, but skip structural zeros in H.
  auto observation_product = [&h](int observation, const double * column, int stride) {
      double value = 0.0;
      if (observation < 3 || observation >= 6) {
        for (int k = 3; k < 7; ++k) {
          value += h[16 * observation + k] * column[stride * k];
        }
      }
      if (observation >= 3) {
        const int k = 7 + observation;
        value += h[16 * observation + k] * column[stride * k];
      }
      return value;
    };
  for (int i = 0; i < 16; ++i) {
    for (int a = 0; a < 9; ++a) {
      ph[9 * i + a] = observation_product(a, &covariance[i], 16);
    }
  }
  for (int a = 0; a < 9; ++a) {
    for (int b = 0; b < 9; ++b) {
      s[9 * a + b] = measurement_covariance[a + 9 * b];
      s[9 * a + b] += observation_product(a, &ph[b], 9);
    }
  }
  if (!innovation_factor(s, 9, l)) {return false;}
  double gain[144]{};
  for (int i = 0; i < 16; ++i) {
    const bool angular = (i >= 3 && i <= 6) || (i >= 10 && i <= 12);
    const int size = angular && consider_acceleration ? 6 : 9;
    double forward[9]{};
    for (int a = 0; a < size; ++a) {
      double v = ph[9 * i + a];
      for (int b = 0; b < a; ++b) {
        v -= l[9 * a + b] * forward[b];
      }
      forward[a] = v / l[9 * a + a];
    }
    for (int a = size - 1; a >= 0; --a) {
      double v = forward[a];
      for (int b = a + 1; b < size; ++b) {
        v -= l[9 * b + a] * gain[9 * i + b];
      }
      gain[9 * i + a] = v / l[9 * a + a];
    }
  }
  if (constrain_tilt_heading) {
    // Gravity and raw acceleration cannot supply world heading. Constrain
    // only their quaternion gain columns; rate measurements can legitimately
    // revise propagated heading through quaternion/rate cross covariance.
    const double yaw_tangent[4]{-z, -y, x, w};
    for (int column = 0; column < 9; ++column) {
      if (column >= 3 && column <= 5) {continue;}
      double component = 0.0;
      for (int row = 0; row < 4; ++row) {
        component += yaw_tangent[row] * gain[9 * (3 + row) + column];
      }
      for (int row = 0; row < 4; ++row) {
        gain[9 * (3 + row) + column] -= yaw_tangent[row] * component;
      }
    }
  }
  State corrected = state;
  Covariance a{}, ap{}, result{};
  for (int i = 0; i < 16; ++i) {
    for (int b = 0; b < 9; ++b) {
      corrected[i] += gain[9 * i + b] * residual[b];
    }
    for (int j0 = 0; j0 < 16; ++j0) {
      a[i + 16 * j0] = i == j0 ? 1.0 : 0.0;
      for (int b = 0; b < 9; ++b) {
        a[i + 16 * j0] -= gain[9 * i + b] * h[16 * b + j0];
      }
    }
  }
  for (int i = 0; i < 16; ++i) {
    for (int j0 = 0; j0 < 16; ++j0) {
      for (int k = 0; k < 16; ++k) {
        ap[i + 16 * j0] += a[i + 16 * k] * covariance[k + 16 * j0];
      }
    }
  }
  double kr[144]{};
  for (int i = 0; i < 16; ++i) {
    for (int b = 0; b < 9; ++b) {
      for (int c = 0; c < 9; ++c) {
        kr[9 * i + b] += gain[9 * i + c] * measurement_covariance[c + 9 * b];
      }
    }
  }
  // Compute each symmetric Joseph entry once. Recomputing both triangles
  // independently can magnify cancellation in K*R*K' into an asymmetric
  // posterior when one packet reports a very large rate covariance.
  for (int i = 0; i < 16; ++i) {
    for (int j0 = 0; j0 <= i; ++j0) {
      for (int k = 0; k < 16; ++k) {
        result[i + 16 * j0] += ap[i + 16 * k] * a[j0 + 16 * k];
      }
      for (int b = 0; b < 9; ++b) {
        result[i + 16 * j0] += kr[9 * i + b] * gain[9 * j0 + b];
      }
      result[j0 + 16 * i] = result[i + 16 * j0];
    }
  }
  if (!finite(corrected)) {return false;}
  normalize_state_covariance(corrected, result, &state);
  state = corrected;
  covariance = result;
  return true;
}

}  // namespace riptide_navigation
