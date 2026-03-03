// Copyright 2025 Analog Devices, Inc.
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

#include "adi_imu/sliding_window_covariance_provider.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <utility>

namespace adi_imu
{
SlidingWindowCovarianceProvider::SlidingWindowCovarianceProvider(
  size_t window_size, size_t min_samples, double min_variance, MotionDetector motion_detector)
: m_window_size(window_size),
  m_min_samples(min_samples),
  m_min_variance(min_variance),
  m_motion_detector(std::move(motion_detector)),
  m_accel_covariance{},
  m_gyro_covariance{},
  m_update_interval(50),
  m_samples_since_update(0)
{
}

void SlidingWindowCovarianceProvider::addSample(const Vec3 & accel, const Vec3 & gyro)
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

  // Add to buffers
  m_accel_samples.push_back(accel);
  m_gyro_samples.push_back(gyro);

  // Maintain window size
  if (m_accel_samples.size() > m_window_size) {
    m_accel_samples.pop_front();
    m_gyro_samples.pop_front();
  }

  // Recompute periodically for efficiency
  m_samples_since_update++;
  if (m_samples_since_update >= m_update_interval && isReady()) {
    recomputeCovariance();
    m_samples_since_update = 0;
  }
}

bool SlidingWindowCovarianceProvider::isReady() const
{
  return m_accel_samples.size() >= m_min_samples;
}

void SlidingWindowCovarianceProvider::recomputeCovariance()
{
  size_t n = m_accel_samples.size();
  if (n < 2) {
    return;
  }

  // Compute means
  Vec3 accel_mean{};
  Vec3 gyro_mean{};

  for (const auto & s : m_accel_samples) {
    accel_mean.x += s.x;
    accel_mean.y += s.y;
    accel_mean.z += s.z;
  }

  for (const auto & s : m_gyro_samples) {
    gyro_mean.x += s.x;
    gyro_mean.y += s.y;
    gyro_mean.z += s.z;
  }

  double dn = static_cast<double>(n);
  accel_mean.x /= dn;
  accel_mean.y /= dn;
  accel_mean.z /= dn;
  gyro_mean.x /= dn;
  gyro_mean.y /= dn;
  gyro_mean.z /= dn;

  // Compute variances
  Vec3 accel_var{};
  Vec3 gyro_var{};

  for (const auto & s : m_accel_samples) {
    accel_var.x += (s.x - accel_mean.x) * (s.x - accel_mean.x);
    accel_var.y += (s.y - accel_mean.y) * (s.y - accel_mean.y);
    accel_var.z += (s.z - accel_mean.z) * (s.z - accel_mean.z);
  }

  for (const auto & s : m_gyro_samples) {
    gyro_var.x += (s.x - gyro_mean.x) * (s.x - gyro_mean.x);
    gyro_var.y += (s.y - gyro_mean.y) * (s.y - gyro_mean.y);
    gyro_var.z += (s.z - gyro_mean.z) * (s.z - gyro_mean.z);
  }

  // Applying Bessel correction for unbiased variance
  double dn1 = static_cast<double>(n - 1);
  accel_var.x = std::max(accel_var.x / dn1, m_min_variance);
  accel_var.y = std::max(accel_var.y / dn1, m_min_variance);
  accel_var.z = std::max(accel_var.z / dn1, m_min_variance);
  gyro_var.x = std::max(gyro_var.x / dn1, m_min_variance);
  gyro_var.y = std::max(gyro_var.y / dn1, m_min_variance);
  gyro_var.z = std::max(gyro_var.z / dn1, m_min_variance);

  m_accel_covariance = {accel_var.x, 0.0, 0.0, 0.0, accel_var.y, 0.0, 0.0, 0.0, accel_var.z};

  m_gyro_covariance = {gyro_var.x, 0.0, 0.0, 0.0, gyro_var.y, 0.0, 0.0, 0.0, gyro_var.z};
}

CovarianceMatrix SlidingWindowCovarianceProvider::getAccelCovariance() const
{
  return m_accel_covariance;
}

CovarianceMatrix SlidingWindowCovarianceProvider::getGyroCovariance() const
{
  return m_gyro_covariance;
}

void SlidingWindowCovarianceProvider::reset()
{
  m_accel_samples.clear();
  m_gyro_samples.clear();
  m_accel_covariance = {};
  m_gyro_covariance = {};
  m_samples_since_update = 0;
}

double SlidingWindowCovarianceProvider::getCalibrationProgress() const
{
  if (m_accel_samples.size() >= m_min_samples) {
    return 1.0;
  }
  return static_cast<double>(m_accel_samples.size()) / static_cast<double>(m_min_samples);
}

}  // namespace adi_imu
