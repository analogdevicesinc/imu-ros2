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

#ifndef ADI_IMU__MOTION_DETECTOR_H_
#define ADI_IMU__MOTION_DETECTOR_H_

#include <cmath>

#include "adi_imu/imu_covariance_interface.h"

namespace adi_imu
{

/**
 * @brief Detects whether the IMU is stationary based on sensor readings.
 *
 * Uses simple thresholding on gyroscope magnitude and accelerometer deviation
 * from gravity to determine stationarity. This allows covariance estimators
 * to only update during stationary periods, avoiding motion-induced variance
 * inflation.
 */
class MotionDetector
{
public:
  // Default thresholds for stationarity detection
  // Gyro threshold: max angular velocity magnitude (rad/s)
  inline static constexpr double DEFAULT_GYRO_THRESHOLD = 0.05;
  // Accel threshold: max deviation from gravity magnitude (m/s^2)
  inline static constexpr double DEFAULT_ACCEL_THRESHOLD = 0.5;
  // Expected gravity magnitude (m/s^2)
  inline static constexpr double GRAVITY_MAGNITUDE = 9.81;

  /**
   * @brief Construct motion detector with configurable thresholds.
   * @param gyro_threshold Max gyro magnitude to consider stationary (rad/s)
   * @param accel_threshold Max accel deviation from gravity (m/s^2)
   */
  explicit MotionDetector(
    double gyro_threshold = DEFAULT_GYRO_THRESHOLD,
    double accel_threshold = DEFAULT_ACCEL_THRESHOLD);

  /**
   * @brief Check if the sensor is stationary based on current readings.
   * @param accel Linear acceleration sample (m/s^2)
   * @param gyro Angular velocity sample (rad/s)
   * @return true if sensor appears stationary
   */
  bool isStationary(const Vec3 & accel, const Vec3 & gyro) const;

  /**
   * @brief Set the gyroscope threshold for stationarity.
   * @param threshold Max angular velocity magnitude (rad/s)
   */
  void setGyroThreshold(double threshold);

  /**
   * @brief Set the accelerometer threshold for stationarity.
   * @param threshold Max deviation from gravity (m/s^2)
   */
  void setAccelThreshold(double threshold);

  /**
   * @brief Get current gyroscope threshold.
   */
  double getGyroThreshold() const { return m_gyro_threshold; }

  /**
   * @brief Get current accelerometer threshold.
   */
  double getAccelThreshold() const { return m_accel_threshold; }

  /**
   * @brief Enable or disable motion detection.
   * When disabled, isStationary() always returns true.
   */
  void setEnabled(bool enabled) { m_enabled = enabled; }

  /**
   * @brief Check if motion detection is enabled.
   */
  bool isEnabled() const { return m_enabled; }

private:
  double m_gyro_threshold;
  double m_accel_threshold;
  bool m_enabled;
};

}  // namespace adi_imu

#endif  // ADI_IMU__MOTION_DETECTOR_H_
