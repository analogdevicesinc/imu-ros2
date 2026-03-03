/*******************************************************************************
 *   @file   kalman_covariance_provider.h
 *   @brief  Kalman filter-based covariance estimation for IMU data.
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

#ifndef ADI_IMU__KALMAN_COVARIANCE_PROVIDER_H_
#define ADI_IMU__KALMAN_COVARIANCE_PROVIDER_H_

#include <cstddef>

#include "adi_imu/imu_covariance_interface.h"
#include "adi_imu/motion_detector.h"

namespace adi_imu
{

/**
 * @brief Kalman filter-based covariance estimator for IMU data.
 *
 * This provider uses a scalar Kalman filter for each axis to estimate
 * the measurement variance. The approach treats variance as a slowly-varying
 * state and uses squared residuals from the running mean as noisy measurements
 * of the true variance.
 *
 * State model: variance_k = variance_{k-1} + w_k, where w_k ~ N(0, Q)
 * Measurement model: z_k = variance_k + v_k, where v_k ~ N(0, R)
 * True variance measurement: y_k = (sample - mean)^2
 *
 * The filter provides adaptive, smoothed variance estimates that respond
 * to changes in sensor noise characteristics while filtering out spurious
 * variations.
 *
 * Parameters:
 * - process_noise_q: How fast the variance can change (larger = more adaptive)
 * - measurement_noise_r: How noisy our variance observations are
 * - initial_variance: Starting estimate for variance
 * - warmup_samples: Samples needed before estimates are considered valid
 * - min_variance: Minimum variance floor to prevent numerical issues
 */
class KalmanCovarianceProvider : public ImuCovarianceInterface
{
public:
  // Default Kalman filter covariance algorithm parameters
  inline static constexpr double DEFAULT_PROCESS_NOISE_Q = 1e-6;
  inline static constexpr double DEFAULT_MEASUREMENT_NOISE_R = 1e-4;
  inline static constexpr double DEFAULT_INITIAL_VARIANCE = 1e-4;
  inline static constexpr size_t DEFAULT_WARMUP_SAMPLES = 100;
  inline static constexpr double DEFAULT_MIN_VARIANCE = 1e-9;

  /**
   * @brief Construct a Kalman filter covariance provider.
   * @param process_noise_q Process noise covariance (variance change rate)
   * @param measurement_noise_r Measurement noise covariance
   * @param initial_variance Initial variance estimate for all axes
   * @param warmup_samples Number of samples before estimates are valid
   * @param min_variance Minimum variance floor
   * @param motion_detector Optional motion detector for stationary filtering
   */
  KalmanCovarianceProvider(
    double process_noise_q = DEFAULT_PROCESS_NOISE_Q,
    double measurement_noise_r = DEFAULT_MEASUREMENT_NOISE_R,
    double initial_variance = DEFAULT_INITIAL_VARIANCE,
    size_t warmup_samples = DEFAULT_WARMUP_SAMPLES, double min_variance = DEFAULT_MIN_VARIANCE,
    MotionDetector motion_detector = MotionDetector());

  ~KalmanCovarianceProvider() override = default;

  void addSample(const Vec3 & accel, const Vec3 & gyro) override;
  bool isReady() const override;
  CovarianceMatrix getAccelCovariance() const override;
  CovarianceMatrix getGyroCovariance() const override;
  void reset() override;
  double getCalibrationProgress() const override;

private:
  /**
   * @brief Per-axis Kalman filter state for variance estimation.
   */
  struct AxisFilter
  {
    double mean;              // Running mean estimate
    double variance;          // Kalman state: estimated variance
    double error_covariance;  // Kalman error covariance (P)

    AxisFilter() : mean(0.0), variance(0.0), error_covariance(1.0) {}
  };

  /**
   * @brief Update a single axis filter with a new sample.
   * @param filter The axis filter to update
   * @param sample The new measurement
   */
  void updateAxisFilter(AxisFilter & filter, double sample);

  /**
   * @brief Build diagonal covariance matrix from axis variances.
   * @param var_x Variance for x-axis
   * @param var_y Variance for y-axis
   * @param var_z Variance for z-axis
   * @return 3x3 diagonal covariance matrix
   */
  CovarianceMatrix buildCovarianceMatrix(double var_x, double var_y, double var_z) const;

  // Kalman filter parameters
  double m_process_noise_q;      // Q: process noise covariance
  double m_measurement_noise_r;  // R: measurement noise covariance
  double m_initial_variance;     // Initial variance estimate
  double m_min_variance;         // Minimum variance floor

  // Motion detector for stationary filtering
  MotionDetector m_motion_detector;

  // Per-axis filters
  AxisFilter m_accel_x;
  AxisFilter m_accel_y;
  AxisFilter m_accel_z;
  AxisFilter m_gyro_x;
  AxisFilter m_gyro_y;
  AxisFilter m_gyro_z;

  // Sample tracking
  size_t m_warmup_samples;
  size_t m_sample_count;

  // Mean estimation parameters (for computing residuals)
  static constexpr double MEAN_ALPHA = 0.01;  // EMA factor for mean tracking
};

}  // namespace adi_imu

#endif  // ADI_IMU__KALMAN_COVARIANCE_PROVIDER_H_
