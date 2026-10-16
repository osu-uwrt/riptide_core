// Copyright 2026 OSU Underwater Robotics Team
// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include "riptide_navigation/estimator_support.hpp"

namespace riptide_navigation
{

// Exact linear Kalman correction for one observed state coordinate. Retain the
// complete covariance and all cross terms. Quaternion normalization/transport
// remains the caller's responsibility, as for the generated correction.
inline bool correct_scalar_observation(
  State & state, Covariance & covariance, int coordinate,
  double measurement, double variance)
{
  if (coordinate < 0 || coordinate >= 16 || !std::isfinite(measurement) ||
    !std::isfinite(variance) || variance < 0.0)
  {
    return false;
  }
  const double prior_variance = covariance[coordinate + 16 * coordinate];
  const double innovation_variance = prior_variance + variance;
  const double residual = measurement - state[coordinate];
  if (prior_variance < 0.0 || !std::isfinite(innovation_variance) || innovation_variance <= 0.0 ||
    !std::isfinite(residual))
  {
    return false;
  }
  std::array<double, 16> cross{}, gain{};
  for (int i = 0; i < 16; ++i) {
    cross[i] = covariance[i + 16 * coordinate];
    gain[i] = cross[i] / innovation_variance;
    if (!std::isfinite(gain[i])) {return false;}
  }
  State corrected = state;
  Covariance posterior = covariance;
  for (int i = 0; i < 16; ++i) {
    corrected[i] += gain[i] * residual;
    if (!std::isfinite(corrected[i])) {return false;}
    for (int j = 0; j <= i; ++j) {
      // Expanded Joseph form: (I-KH)P(I-KH)' + K R K'.
      double value = covariance[i + 16 * j] - gain[i] * cross[j] -
        cross[i] * gain[j] + gain[i] * prior_variance * gain[j];
      // Add measurement noise separately, so rounding Pcc + R to Pcc cannot
      // erase the posterior noise. Observed cross terms also avoid 1-Kc.
      value += gain[i] * variance * gain[j];
      if (i == coordinate || j == coordinate) {
        const int other = i == coordinate ? j : i;
        value = cross[other] * (variance / innovation_variance);
      }
      if (!std::isfinite(value)) {return false;}
      posterior[i + 16 * j] = posterior[j + 16 * i] = value;
    }
  }
  state = corrected;
  covariance = posterior;
  return true;
}

}  // namespace riptide_navigation
