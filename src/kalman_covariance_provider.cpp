/*******************************************************************************
 *   @file   kalman_covariance_provider.cpp
 *   @brief  Implementation of Kalman filter-based covariance estimation.
 *   @author Tudor Alinei
 *******************************************************************************/
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

#include "adi_imu/kalman_covariance_provider.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace adi_imu
{

KalmanCovarianceProvider::KalmanCovarianceProvider(
  double process_noise_q, double measurement_noise_r, double initial_variance,
  size_t warmup_samples, double min_variance, MotionDetector motion_detector)
: m_process_noise_q(process_noise_q),
  m_measurement_noise_r(measurement_noise_r),
  m_initial_variance(initial_variance),
  m_min_variance(min_variance),
  m_motion_detector(std::move(motion_detector)),
  m_warmup_samples(warmup_samples),
  m_sample_count(0)
{
  reset();
}

void KalmanCovarianceProvider::reset()
{
  m_sample_count = 0;

  // Initialize all axis filters
  auto initFilter = [this](AxisFilter & f) {
    f.mean = 0.0;
    f.variance = m_initial_variance;
    f.error_covariance = m_initial_variance;  // Initial uncertainty in variance estimate
  };

  initFilter(m_accel_x);
  initFilter(m_accel_y);
  initFilter(m_accel_z);
  initFilter(m_gyro_x);
  initFilter(m_gyro_y);
  initFilter(m_gyro_z);
}

void KalmanCovarianceProvider::updateAxisFilter(AxisFilter & filter, double sample)
{
  // Guard against NaN/Inf inputs
  if (!std::isfinite(sample)) {
    return;
  }

  // Update running mean using exponential moving average
  // For first sample, initialize mean directly
  if (m_sample_count == 0) {
    filter.mean = sample;
  } else {
    filter.mean = (1.0 - MEAN_ALPHA) * filter.mean + MEAN_ALPHA * sample;
  }

  // Compute squared residual as measurement of variance
  double residual = sample - filter.mean;
  double measurement = residual * residual;

  // === Kalman Filter Update ===

  // Prediction step:
  // State prediction: variance_predicted = variance (no change model)
  // Error covariance prediction: P_predicted = P + Q
  double P_predicted = filter.error_covariance + m_process_noise_q;

  // Update step:
  // Kalman gain: K = P_predicted / (P_predicted + R)
  double K = P_predicted / (P_predicted + m_measurement_noise_r);

  // State update: variance = variance + K * (measurement - variance)
  filter.variance = filter.variance + K * (measurement - filter.variance);

  // Error covariance update: P = (1 - K) * P_predicted (is equiv to the bellow form = Joseph form --more stable--)
  filter.error_covariance = (1.0 - K) * P_predicted * (1.0 - K) + K * m_measurement_noise_r * K;

  // Enforce minimum variance floor
  filter.variance = std::max(filter.variance, m_min_variance);

  // Ensure error covariance stays positive
  filter.error_covariance = std::max(filter.error_covariance, m_min_variance);
}

void KalmanCovarianceProvider::addSample(const Vec3 & accel, const Vec3 & gyro)
{
  // Skip non-stationary samples to avoid motion-induced variance inflation
  if (!m_motion_detector.isStationary(accel, gyro)) {
    return;
  }

  // Update each axis filter
  updateAxisFilter(m_accel_x, accel.x);
  updateAxisFilter(m_accel_y, accel.y);
  updateAxisFilter(m_accel_z, accel.z);
  updateAxisFilter(m_gyro_x, gyro.x);
  updateAxisFilter(m_gyro_y, gyro.y);
  updateAxisFilter(m_gyro_z, gyro.z);

  // Increment sample count (cap to prevent overflow)
  if (m_sample_count < m_warmup_samples) {
    ++m_sample_count;
  }
}

bool KalmanCovarianceProvider::isReady() const { return m_sample_count >= m_warmup_samples; }

double KalmanCovarianceProvider::getCalibrationProgress() const
{
  if (m_warmup_samples == 0) {
    return 1.0;
  }
  return std::min(1.0, static_cast<double>(m_sample_count) / static_cast<double>(m_warmup_samples));
}

CovarianceMatrix KalmanCovarianceProvider::buildCovarianceMatrix(
  double var_x, double var_y, double var_z) const
{
  // Build diagonal covariance matrix (assuming independent axes)
  // Layout: [xx, xy, xz, yx, yy, yz, zx, zy, zz]
  return {var_x, 0.0, 0.0, 0.0, var_y, 0.0, 0.0, 0.0, var_z};
}

CovarianceMatrix KalmanCovarianceProvider::getAccelCovariance() const
{
  return buildCovarianceMatrix(m_accel_x.variance, m_accel_y.variance, m_accel_z.variance);
}

CovarianceMatrix KalmanCovarianceProvider::getGyroCovariance() const
{
  return buildCovarianceMatrix(m_gyro_x.variance, m_gyro_y.variance, m_gyro_z.variance);
}

}  // namespace adi_imu
