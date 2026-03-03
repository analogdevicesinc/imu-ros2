// Copyright 2026 Analog Devices, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "adi_imu/ewma_covariance_provider.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace adi_imu
{
EwmaCovarianceProvider::EwmaCovarianceProvider(
  double alpha, size_t warmup_samples, double min_variance, MotionDetector motion_detector)
: m_alpha(alpha),
  m_warmup_samples(warmup_samples),
  m_sample_count(0),
  m_min_variance(min_variance),
  m_motion_detector(std::move(motion_detector)),
  m_accel_mean{},
  m_accel_var{},
  m_gyro_mean{},
  m_gyro_var{}
{
}

void EwmaCovarianceProvider::addSample(const Vec3 & accel, const Vec3 & gyro)
{
  // Guard against NaN from sensor
  if (
    std::isnan(accel.x) || std::isnan(accel.y) || std::isnan(accel.z) || std::isnan(gyro.x) ||
    std::isnan(gyro.y) || std::isnan(gyro.z)) {
    return;
  }

  // Skip non-stationary samples to avoid motion-induced variance inflation
  if (!m_motion_detector.isStationary(accel, gyro)) {
    return;
  }

  // Only increment until warmup completes - prevents overflow
  if (m_sample_count < m_warmup_samples) {
    m_sample_count++;
  }

  if (m_sample_count == 1) {
    // Initialize with first sample
    m_accel_mean = accel;
    m_gyro_mean = gyro;
    return;
  }

  // EWMA mean update: mean_t = alpha * x_t + (1-alpha) * mean_{t-1}
  auto updateMean = [this](double x, double & mean) {
    mean = m_alpha * x + (1.0 - m_alpha) * mean;
  };

  // EWMA variance update: var_t = alpha * (x_t - mean_t)^2 + (1-alpha) * var_{t-1}
  auto updateVar = [this](double x, double mean, double & var) {
    double diff = x - mean;
    var = m_alpha * (diff * diff) + (1.0 - m_alpha) * var;
  };

  // Update accelerometer
  updateVar(accel.x, m_accel_mean.x, m_accel_var.x);
  updateVar(accel.y, m_accel_mean.y, m_accel_var.y);
  updateVar(accel.z, m_accel_mean.z, m_accel_var.z);
  updateMean(accel.x, m_accel_mean.x);
  updateMean(accel.y, m_accel_mean.y);
  updateMean(accel.z, m_accel_mean.z);

  // Update gyroscope
  updateVar(gyro.x, m_gyro_mean.x, m_gyro_var.x);
  updateVar(gyro.y, m_gyro_mean.y, m_gyro_var.y);
  updateVar(gyro.z, m_gyro_mean.z, m_gyro_var.z);
  updateMean(gyro.x, m_gyro_mean.x);
  updateMean(gyro.y, m_gyro_mean.y);
  updateMean(gyro.z, m_gyro_mean.z);
}

bool EwmaCovarianceProvider::isReady() const { return m_sample_count >= m_warmup_samples; }

CovarianceMatrix EwmaCovarianceProvider::getAccelCovariance() const
{
  return {std::max(m_accel_var.x, m_min_variance), 0.0, 0.0, 0.0,
          std::max(m_accel_var.y, m_min_variance), 0.0, 0.0, 0.0,
          std::max(m_accel_var.z, m_min_variance)};
}

CovarianceMatrix EwmaCovarianceProvider::getGyroCovariance() const
{
  return {std::max(m_gyro_var.x, m_min_variance), 0.0, 0.0, 0.0,
          std::max(m_gyro_var.y, m_min_variance), 0.0, 0.0, 0.0,
          std::max(m_gyro_var.z, m_min_variance)};
}

void EwmaCovarianceProvider::reset()
{
  m_sample_count = 0;
  m_accel_mean = Vec3();
  m_accel_var = Vec3();
  m_gyro_mean = Vec3();
  m_gyro_var = Vec3();
}

double EwmaCovarianceProvider::getCalibrationProgress() const
{
  if (m_sample_count >= m_warmup_samples) {
    return 1.0;
  }
  return static_cast<double>(m_sample_count) / static_cast<double>(m_warmup_samples);
}
}  // namespace adi_imu