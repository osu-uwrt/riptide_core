// Copyright 2026 OSU Underwater Robotics Team
// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

namespace riptide_navigation
{

using State = std::array<double, 16>;
using Covariance = std::array<double, 256>;  // column-major, matching generated code

inline bool finite(const State & x)
{
  return std::all_of(x.begin(), x.end(), [](double value) {return std::isfinite(value);});
}

inline bool normalize_quaternion(State & x, const State * reference = nullptr)
{
  const double norm = std::hypot(std::hypot(x[3], x[4]), std::hypot(x[5], x[6]));
  if (!std::isfinite(norm) || norm < 1e-12) {
    return false;
  }
  for (std::size_t i = 3; i < 7; ++i) {
    x[i] /= norm;
  }
  if (reference != nullptr) {
    double dot = 0.0;
    for (std::size_t i = 3; i < 7; ++i) {
      dot += x[i] * (*reference)[i];
    }
    if (dot < 0.0) {
      for (std::size_t i = 3; i < 7; ++i) {
        x[i] = -x[i];
      }
    }
  }
  return true;
}

// A gravity-direction measurement is invariant to left multiplication by a
// world-Z rotation. Use that exact gauge freedom to retain the heading carried
// into the correction while accepting the corrected tilt.
inline State remove_world_yaw_correction(const State & before, State corrected)
{
  State reference = before;
  if (!normalize_quaternion(reference) || !normalize_quaternion(corrected, &reference)) {
    return before;
  }

  const double bw = reference[3], bx = reference[4];
  const double by = reference[5], bz = reference[6];
  const double aw = corrected[3], ax = corrected[4];
  const double ay = corrected[5], az = corrected[6];

  const double before_heading_norm = std::hypot(bw, bz);
  const double corrected_heading_norm = std::hypot(aw, az);
  const double absolute_half_delta = std::atan2(bz, bw) - std::atan2(az, aw);

  // The nearest member of corrected's world-Z gauge orbit is well conditioned
  // even at inverted attitude. Blend toward it as the absolute heading chart
  // approaches its singularity. Every point in the blend is still a world-Z
  // rotation of corrected, so the accepted gravity direction is unchanged.
  const double orbit_dot = bw * aw + bx * ax + by * ay + bz * az;
  const double orbit_cross = -bw * az - bx * ay + by * ax + bz * aw;
  const double nearest_half_delta = std::atan2(orbit_cross, orbit_dot);
  const double chart_norm = std::min(before_heading_norm, corrected_heading_norm);
  const double blend_coordinate = std::clamp((chart_norm - 0.1) / 0.15, 0.0, 1.0);
  const double absolute_weight =
    blend_coordinate * blend_coordinate * (3.0 - 2.0 * blend_coordinate);
  const double delta_difference = std::atan2(
    std::sin(absolute_half_delta - nearest_half_delta),
    std::cos(absolute_half_delta - nearest_half_delta));
  const double half_delta = nearest_half_delta + absolute_weight * delta_difference;
  const double dw = std::cos(half_delta);
  const double dz = std::sin(half_delta);
  corrected[3] = dw * aw - dz * az;
  corrected[4] = dw * ax - dz * ay;
  corrected[5] = dw * ay + dz * ax;
  corrected[6] = dw * az + dz * aw;
  normalize_quaternion(corrected, &reference);
  return corrected;
}

inline std::array<double, 9> rotation(const State & x)
{
  const double w = x[3], qx = x[4], qy = x[5], qz = x[6];
  return {1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * w), 2 * (qx * qz + qy * w),
    2 * (qx * qy + qz * w), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * w),
    2 * (qx * qz - qy * w), 2 * (qy * qz + qx * w), 1 - 2 * (qx * qx + qy * qy)};
}

inline State predict_state(State x, double dt)
{
  dt = std::clamp(dt, 0.0, 0.1);
  normalize_quaternion(x);
  const auto r = rotation(x);
  const std::array<double, 3> v{x[7], x[8], x[9]};
  const std::array<double, 3> w{x[10], x[11], x[12]};
  const std::array<double, 3> a{x[13], x[14], x[15]};
  for (std::size_t row = 0; row < 3; ++row) {
    double rv = 0.0, ra = 0.0;
    for (std::size_t col = 0; col < 3; ++col) {
      rv += r[row * 3 + col] * v[col];
      ra += r[row * 3 + col] * a[col];
    }
    x[row] += rv * dt + 0.5 * ra * dt * dt;
  }
  const double angle = std::hypot(std::hypot(w[0], w[1]), w[2]) * dt;
  std::array<double, 4> dq{1.0, 0.5 * w[0] * dt, 0.5 * w[1] * dt, 0.5 * w[2] * dt};
  if (angle >= 1e-9) {
    const double scale = std::sin(0.5 * angle) * dt / angle;
    dq = {std::cos(0.5 * angle), scale * w[0], scale * w[1], scale * w[2]};
  }
  const std::array<double, 4> q{x[3], x[4], x[5], x[6]};
  x[3] = q[0] * dq[0] - q[1] * dq[1] - q[2] * dq[2] - q[3] * dq[3];
  x[4] = q[0] * dq[1] + dq[0] * q[1] + q[2] * dq[3] - q[3] * dq[2];
  x[5] = q[0] * dq[2] + dq[0] * q[2] + q[3] * dq[1] - q[1] * dq[3];
  x[6] = q[0] * dq[3] + dq[0] * q[3] + q[1] * dq[2] - q[2] * dq[1];
  x[7] += (a[0] - (w[1] * v[2] - w[2] * v[1])) * dt;
  x[8] += (a[1] - (w[2] * v[0] - w[0] * v[2])) * dt;
  x[9] += (a[2] - (w[0] * v[1] - w[1] * v[0])) * dt;
  normalize_quaternion(x);
  return x;
}

inline Covariance predict_covariance(
  const State & state, const Covariance & covariance, double dt,
  const std::vector<double> & process_noise_diag)
{
  double f[256]{};
  const State nominal = predict_state(state, dt);
  for (int col = 0; col < 16; ++col) {
    const double h = 1e-6 * std::max(1.0, std::abs(state[col]));
    State plus = state;
    State minus = state;
    plus[col] += h;
    minus[col] -= h;
    plus = predict_state(plus, dt);
    minus = predict_state(minus, dt);
    double plus_dot = 0.0;
    double minus_dot = 0.0;
    for (int i = 3; i < 7; ++i) {
      plus_dot += plus[i] * nominal[i];
      minus_dot += minus[i] * nominal[i];
    }
    if (plus_dot < 0.0) {
      for (int i = 3; i < 7; ++i) {
        plus[i] = -plus[i];
      }
    }
    if (minus_dot < 0.0) {
      for (int i = 3; i < 7; ++i) {
        minus[i] = -minus[i];
      }
    }
    for (int row = 0; row < 16; ++row) {
      f[row + 16 * col] = (plus[row] - minus[row]) / (2.0 * h);
    }
  }
  Covariance fp{};
  Covariance result{};
  for (int row = 0; row < 16; ++row) {
    for (int col = 0; col < 16; ++col) {
      for (int k = 0; k < 16; ++k) {
        fp[row + 16 * col] += f[row + 16 * k] * covariance[k + 16 * col];
      }
    }
  }
  for (int row = 0; row < 16; ++row) {
    for (int col = 0; col < 16; ++col) {
      for (int k = 0; k < 16; ++k) {
        result[row + 16 * col] += fp[row + 16 * k] * f[col + 16 * k];
      }
    }
  }
  const double scale = std::max(0.0, dt);
  for (int i = 0; i < 16 && i < static_cast<int>(process_noise_diag.size()); ++i) {
    if (i < 3 || i > 6) {
      result[i + 16 * i] += process_noise_diag[i] * scale;
    }
  }
  // The configured attitude terms are angular-error spectral densities. Map
  // them into quaternion coordinates with dq = 0.5 G(q) dtheta.
  if (process_noise_diag.size() >= 6) {
    const double w = nominal[3], x = nominal[4], y = nominal[5], z = nominal[6];
    const double g[12] = {-x, -y, -z, w, -z, y, z, w, -x, -y, x, w};
    for (int row = 0; row < 4; ++row) {
      for (int col = 0; col < 4; ++col) {
        double value = 0.0;
        for (int axis = 0; axis < 3; ++axis) {
          value += g[row * 3 + axis] * process_noise_diag[3 + axis] *
            g[col * 3 + axis];
        }
        result[(3 + row) + 16 * (3 + col)] += 0.25 * scale * value;
      }
    }
  }
  return result;
}

inline void normalize_state_covariance(State & x, Covariance & p, const State * reference = nullptr)
{
  const double q[4]{x[3], x[4], x[5], x[6]};
  const double norm = std::hypot(std::hypot(q[0], q[1]), std::hypot(q[2], q[3]));
  if (norm < 1e-12 || !std::isfinite(norm)) {
    x[3] = 1.0; x[4] = x[5] = x[6] = 0.0;
    return;
  }
  double n[4];
  for (int i = 0; i < 4; ++i) {
    n[i] = q[i] / norm;
  }
  double j[16]{};
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      j[row + 4 * col] = ((row == col ? 1.0 : 0.0) - n[row] * n[col]) / norm;
    }
  }

  Covariance tmp = p;
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 16; ++col) {
      double value = 0.0;
      for (int k = 0; k < 4; ++k) {
        value += j[row + 4 * k] * p[(k + 3) + 16 * col];
      }
      tmp[(row + 3) + 16 * col] = value;
    }
  }
  p = tmp;
  for (int row = 0; row < 16; ++row) {
    for (int col = 0; col < 4; ++col) {
      double value = 0.0;
      for (int k = 0; k < 4; ++k) {
        value += tmp[row + 16 * (k + 3)] * j[col + 4 * k];
      }
      p[row + 16 * (col + 3)] = value;
    }
  }
  bool sign_flip = false;
  if (reference != nullptr) {
    double dot = 0.0;
    for (int i = 0; i < 4; ++i) {
      dot += n[i] * (*reference)[3 + i];
    }
    sign_flip = dot < 0.0;
  }
  normalize_quaternion(x, reference);
  if (sign_flip) {
    for (int q_index = 3; q_index < 7; ++q_index) {
      for (int other = 0; other < 16; ++other) {
        if (other < 3 || other > 6) {
          p[q_index + 16 * other] = -p[q_index + 16 * other];
          p[other + 16 * q_index] = -p[other + 16 * q_index];
        }
      }
    }
  }
}

inline bool covariance_is_psd(const Covariance & input, double tolerance = 1e-9)
{
  double l[256]{};
  for (int i = 0; i < 16; ++i) {
    for (int j = 0; j <= i; ++j) {
      const double aij = 0.5 * (input[i + 16 * j] + input[j + 16 * i]);
      double sum = aij;
      for (int k = 0; k < j; ++k) {
        sum -= l[i + 16 * k] * l[j + 16 * k];
      }
      if (i == j) {
        if (sum < -tolerance || !std::isfinite(sum)) {return false;}
        l[i + 16 * j] = std::sqrt(std::max(0.0, sum));
      } else if (l[j + 16 * j] > tolerance) {
        l[i + 16 * j] = sum / l[j + 16 * j];
      } else if (std::abs(sum) > tolerance) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace riptide_navigation
