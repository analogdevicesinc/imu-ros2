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

#ifndef ADI_IMU__EWMA_WINDOW_COVARIANCE_PROVIDER_H_
#define ADI_IMU__EWMA_WINDOW_COVARIANCE_PROVIDER_H_

#include <cstddef>

#include "adi_imu/imu_covariance_interface.h"
#include "adi_imu/motion_detector.h"

namespace adi_imu
{
/**
    * @brief Exponentially Weighted Moving Average covariance estimator.
    *
    * Computes variance using exponential smoothing:
    *  variance_t = alpha * (x_t - mean_t)^2 + (1-alpha) * variance_{t-1}
    *
    * Adapts continuously with O(1) memory. Alpha controls responsiveness:
    * - Higher alpha (0.1): Fast adaptation, more sensitivity
    * - Lower alpha (0.01): Slow adaptation, smoother estimates
     */

class EwmaCovarianceProvider : public ImuCovarianceInterface
{
public:
  // Default EWMA covariance estimator parameters
  inline static constexpr double DEFAULT_ALPHA = 0.02;
  inline static constexpr size_t DEFAULT_WARMUP_SAMPLES = 100;
  inline static constexpr double DEFAULT_MIN_VARIANCE = 1e-9;

  /**
   * @brief Construct EWMA covariance estimator.
   * @param alpha Smoothing factor (0 < alpha < 1). Typical 0.01-0.1
   * @param warmup_samples Samples before covariance is considered valid.
   * @param min_variance Minimum variance floor.
   * @param motion_detector Optional motion detector for stationary filtering.
   */
  explicit EwmaCovarianceProvider(
    double alpha = DEFAULT_ALPHA, size_t warmup_samples = DEFAULT_WARMUP_SAMPLES,
    double min_variance = DEFAULT_MIN_VARIANCE, MotionDetector motion_detector = MotionDetector());

  void addSample(const Vec3 & accel, const Vec3 & gyro) override;
  bool isReady() const override;
  CovarianceMatrix getAccelCovariance() const override;
  CovarianceMatrix getGyroCovariance() const override;
  void reset() override;
  double getCalibrationProgress() const override;

private:
  double m_alpha;
  size_t m_warmup_samples;
  size_t m_sample_count;
  double m_min_variance;

  // Motion detector for stationary filtering
  MotionDetector m_motion_detector;

  // EWMA state (mean and variance for each axis)
  Vec3 m_accel_mean;
  Vec3 m_accel_var;
  Vec3 m_gyro_mean;
  Vec3 m_gyro_var;
};
}  // namespace adi_imu

#endif  // ADI_IMU__EWMA_WINDOW_COVARIANCE_PROVIDER_H_