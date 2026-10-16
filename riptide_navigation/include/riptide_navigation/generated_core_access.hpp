// Copyright 2026 OSU Underwater Robotics Team
// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include "riptide_navigation/estimator_support.hpp"
#include "talos_ekf.h"
#include <algorithm>
#include <limits>
#include <stdexcept>
#include <type_traits>

namespace riptide_navigation
{

// Embedded Coder's supported InternalMemberVisibility=public interface exposes
// the EKF data stores. Fail compilation if regeneration changes their contract.
static_assert(std::extent_v<decltype(talos_ekf::DW::x)> == 16);
static_assert(std::extent_v<decltype(talos_ekf::DW::P_i)> == 256);

inline void copy_generated_core_state(talos_ekf & destination, talos_ekf & source)
{
  // Copy the public generated data stores, including square-root covariance
  // and execution-order flags, between step() calls. Both current targets have
  // no owning/self pointers or private evolving state beyond the RTModel.
  // errorStatus is a non-owning pointer; its storage must outlive both cores.
  static_assert(std::is_trivially_copyable_v<decltype(source.rtDW)>);
  static_assert(std::is_trivially_copyable_v<decltype(source.rtU)>);
  static_assert(std::is_trivially_copyable_v<decltype(source.rtY)>);
  static_assert(std::is_trivially_copyable_v<std::remove_reference_t<decltype(*source.getRTM())>>);
  destination.rtDW = source.rtDW;
  destination.rtU = source.rtU;
  destination.rtY = source.rtY;
  *destination.getRTM() = *source.getRTM();
}

inline void read_generated_posterior(
  const talos_ekf & core, State & state, Covariance & covariance)
{
  std::copy(std::begin(core.rtDW.x), std::end(core.rtDW.x), state.begin());
  // Despite its name, P_i is the square-root factor S, not P. The EKF block
  // stores P=S*S' (column-major) and QR-factorizes S after each prediction.
  covariance.fill(0.0);
  for (int i = 0; i < 16; ++i) {
    for (int j = 0; j <= i; ++j) {
      double value = 0.0;
      for (int k = 0; k < 16; ++k) {
        value += core.rtDW.P_i[i + 16 * k] * core.rtDW.P_i[j + 16 * k];
      }
      covariance[i + 16 * j] = covariance[j + 16 * i] = value;
    }
  }
}

inline bool posterior_square_root(const Covariance & covariance, Covariance & factor)
{
  // Pivoted Cholesky on a diagonally scaled correlation matrix accepts the
  // quaternion's null radial direction without adding artificial variance.
  // Scaling prevents position units from hiding small angular uncertainties.
  constexpr double tolerance = 256 * std::numeric_limits<double>::epsilon();
  double scale[16]{}, diagonal[16]{};
  bool selected[16]{};
  Covariance correlation{}, root{};
  for (int i = 0; i < 16; ++i) {
    const double variance = covariance[i + 16 * i];
    if (!std::isfinite(variance) || variance < 0.0) {return false;}
    scale[i] = std::sqrt(variance);
    diagonal[i] = variance > 0.0 ? 1.0 : 0.0;
  }
  for (int i = 0; i < 16; ++i) {
    for (int j = 0; j < 16; ++j) {
      const double value = covariance[i + 16 * j];
      if (!std::isfinite(value)) {return false;}
      if (scale[i] == 0.0 || scale[j] == 0.0) {
        if (value != 0.0) {return false;}
      } else {
        correlation[i + 16 * j] = value / scale[i] / scale[j];
        if (std::abs(value - covariance[j + 16 * i]) / scale[i] / scale[j] > tolerance) {
          return false;
        }
      }
    }
  }
  for (int column = 0; column < 16; ++column) {
    int pivot = -1;
    for (int i = 0; i < 16; ++i) {
      if (!selected[i] && (pivot<0 || diagonal[i]> diagonal[pivot])) {pivot = i;}
      if (diagonal[i] < -tolerance) {return false;}
    }
    if (pivot < 0 || diagonal[pivot] <= tolerance) {break;}
    selected[pivot] = true;
    root[pivot + 16 * column] = std::sqrt(diagonal[pivot]);
    for (int i = 0; i < 16; ++i) {
      if (selected[i]) {continue;}
      double value = correlation[i + 16 * pivot];
      for (int k = 0; k < column; ++k) {
        value -= root[i + 16 * k] * root[pivot + 16 * k];
      }
      root[i + 16 * column] = value / root[pivot + 16 * column];
      diagonal[i] -= root[i + 16 * column] * root[i + 16 * column];
    }
  }
  // Full-rank Cholesky already verifies PSD. At a null pivot, verify only the
  // remaining Schur block: zero diagonals with nonzero cross terms are invalid.
  for (int i = 0; i < 16; ++i) {
    for (int j = 0; j < 16; ++j) {
      if (selected[i] || selected[j]) {continue;}
      double value = 0.0;
      for (int k = 0; k < 16; ++k) {
        value += root[i + 16 * k] * root[j + 16 * k];
      }
      if (std::abs(value - correlation[i + 16 * j]) > 4 * tolerance) {return false;}
    }
  }
  for (int i = 0; i < 16; ++i) {
    for (int j = 0; j < 16; ++j) {
      factor[i + 16 * j] = scale[i] * root[i + 16 * j];
    }
  }
  return true;
}

inline void write_generated_posterior(
  talos_ekf & core, const State & state, const Covariance & covariance)
{
  // Do not touch generated execution-order flags. Call only between step()
  // calls, under the adapter's filter mutex. Synchronize rtY for reset callers
  // that capture the output before another generated step has occurred.
  Covariance factor{};
  if (!posterior_square_root(covariance, factor)) {
    throw std::invalid_argument("Generated EKF posterior covariance must be symmetric PSD");
  }
  std::copy(state.begin(), state.end(), core.rtDW.x);
  std::copy(factor.begin(), factor.end(), core.rtDW.P_i);
  std::copy(state.begin(), state.end(), core.rtY.state);
  // Covariance remains in the public square-root posterior data store.
}

}  // namespace riptide_navigation
