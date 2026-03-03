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

#ifndef ADI_IMU__SLIDING_WINDOW_COVARIANCE_PROVIDER_H_
#define ADI_IMU__SLIDING_WINDOW_COVARIANCE_PROVIDER_H_

#include <cstddef>
#include <deque>

#include "adi_imu/imu_covariance_interface.h"
#include "adi_imu/motion_detector.h"

namespace adi_imu
{
/**
    * @brief Computes covariances over a sliding window of recent samples.
    *
    * This provider maintains a fixed-size window of samples and continuously
    * updates the covariance estimate. Useful for adaptive covariance that 
    * responds to changing sensor noise characteristics.
     */

class SlidingWindowCovarianceProvider : public ImuCovarianceInterface
{
public:
  /**
            * @brief Default Sliding-window covariance algorithm parameters
             */
  inline static constexpr size_t DEFAULT_WINDOW_SIZE = 500;
  inline static constexpr size_t DEFAULT_MIN_SAMPLES = 100;
  inline static constexpr double DEFAULT_MIN_VARIANCE = 1e-9;

  /**
   * @brief Construct sliding window covariance estimator.
   * @param window_size Number of samples to keep in window.
   * @param min_samples Minimum samples before covariance is valid.
   * @param min_variance Minimum variance floor.
   * @param motion_detector Optional motion detector for stationary filtering.
   */
  explicit SlidingWindowCovarianceProvider(
    size_t window_size = DEFAULT_WINDOW_SIZE, size_t min_samples = DEFAULT_MIN_SAMPLES,
    double min_variance = DEFAULT_MIN_VARIANCE, MotionDetector motion_detector = MotionDetector());

  void addSample(const Vec3 & accel, const Vec3 & gyro) override;
  bool isReady() const override;
  CovarianceMatrix getAccelCovariance() const override;
  CovarianceMatrix getGyroCovariance() const override;
  void reset() override;
  double getCalibrationProgress() const override;

private:
  void recomputeCovariance();

  size_t m_window_size;
  size_t m_min_samples;
  double m_min_variance;

  // Motion detector for stationary filtering
  MotionDetector m_motion_detector;

  // Circular buffers for samples
  std::deque<Vec3> m_accel_samples;
  std::deque<Vec3> m_gyro_samples;

  // Cached covariance (periodically recomputed)
  CovarianceMatrix m_accel_covariance;
  CovarianceMatrix m_gyro_covariance;

  // Recompute every N samples for efficiency
  size_t m_update_interval;
  size_t m_samples_since_update;
};
}  // namespace adi_imu

#endif  // ADI_IMU__SLIDING_WINDOW_COVARIANCE_PROVIDER_H_