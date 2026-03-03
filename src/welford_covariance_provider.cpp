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

#include "adi_imu/welford_covariance_provider.h"

#include <algorithm>
#include <cmath>

namespace adi_imu
{
WelfordCovarianceProvider::WelfordCovarianceProvider(
  size_t calibration_samples, double min_variance)
: m_target_samples(calibration_samples),
  m_sample_count(0),
  m_min_variance(min_variance),
  m_accel_mean{},
  m_accel_M2{},
  m_gyro_mean{},
  m_gyro_M2{},
  m_accel_covariance{},
  m_gyro_covariance{},
  m_calibration_complete(false)
{
}

void WelfordCovarianceProvider::updateWelford(double value, double & mean, double & M2, size_t n)
{
  double delta = value - mean;
  mean += delta / static_cast<double>(n);
  double delta2 = value - mean;
  M2 += delta * delta2;
}

double WelfordCovarianceProvider::computeVariance(double M2, size_t n) const
{
  if (n < 2) {
    return m_min_variance;
  }
  // Using unbiased sample variance
  double variance = M2 / static_cast<double>(n - 1);
  return std::max(variance, m_min_variance);
}

void WelfordCovarianceProvider::addSample(const Vec3 & accel, const Vec3 & gyro)
{
  // Guard against NaN from sensor
  if (
    std::isnan(accel.x) || std::isnan(accel.y) || std::isnan(accel.z) || std::isnan(gyro.x) ||
    std::isnan(gyro.y) || std::isnan(gyro.z)) {
    return;
  }

  if (m_calibration_complete) {
    return;
  }

  m_sample_count++;
  size_t n = m_sample_count;

  // Update accelerometer statistics
  updateWelford(accel.x, m_accel_mean.x, m_accel_M2.x, n);
  updateWelford(accel.y, m_accel_mean.y, m_accel_M2.y, n);
  updateWelford(accel.z, m_accel_mean.z, m_accel_M2.z, n);

  // Update gyro statistics
  updateWelford(gyro.x, m_gyro_mean.x, m_gyro_M2.x, n);
  updateWelford(gyro.y, m_gyro_mean.y, m_gyro_M2.y, n);
  updateWelford(gyro.z, m_gyro_mean.z, m_gyro_M2.z, n);

  // Check if calibration is complete
  if (m_target_samples <= m_sample_count) {
    // Freeze covariance as diag. matrices
    double ax_var = computeVariance(m_accel_M2.x, n);
    double ay_var = computeVariance(m_accel_M2.y, n);
    double az_var = computeVariance(m_accel_M2.z, n);

    double gx_var = computeVariance(m_gyro_M2.x, n);
    double gy_var = computeVariance(m_gyro_M2.y, n);
    double gz_var = computeVariance(m_gyro_M2.z, n);

    m_accel_covariance = {ax_var, 0.0, 0.0, 0.0, ay_var, 0.0, 0.0, 0.0, az_var};

    m_gyro_covariance = {gx_var, 0.0, 0.0, 0.0, gy_var, 0.0, 0.0, 0.0, gz_var};

    m_calibration_complete = true;
  }
}

bool WelfordCovarianceProvider::isReady() const { return m_calibration_complete; }

CovarianceMatrix WelfordCovarianceProvider::getAccelCovariance() const
{
  return m_accel_covariance;
}

CovarianceMatrix WelfordCovarianceProvider::getGyroCovariance() const { return m_gyro_covariance; }

void WelfordCovarianceProvider::reset()
{
  m_sample_count = 0;
  m_accel_mean = Vec3();
  m_accel_M2 = Vec3();
  m_gyro_mean = Vec3();
  m_gyro_M2 = Vec3();
  m_accel_covariance = {};
  m_gyro_covariance = {};
  m_calibration_complete = false;
}

double WelfordCovarianceProvider::getCalibrationProgress() const
{
  if (m_calibration_complete) {
    return 1.0;
  }
  return static_cast<double>(m_sample_count) / static_cast<double>(m_target_samples);
}

}  // namespace adi_imu