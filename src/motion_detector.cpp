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

#include "adi_imu/motion_detector.h"

#include <cmath>

namespace adi_imu
{

MotionDetector::MotionDetector(double gyro_threshold, double accel_threshold)
: m_gyro_threshold(gyro_threshold), m_accel_threshold(accel_threshold), m_enabled(true)
{
}

bool MotionDetector::isStationary(const Vec3 & accel, const Vec3 & gyro) const
{
  // If motion detection is disabled, always return stationary
  if (!m_enabled) {
    return true;
  }

  // Check gyroscope magnitude
  double gyro_magnitude = std::sqrt(gyro.x * gyro.x + gyro.y * gyro.y + gyro.z * gyro.z);
  if (gyro_magnitude > m_gyro_threshold) {
    return false;
  }

  // Check accelerometer deviation from gravity
  // When stationary, accel magnitude should be close to gravity
  double accel_magnitude = std::sqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
  double accel_deviation = std::abs(accel_magnitude - GRAVITY_MAGNITUDE);
  if (accel_deviation > m_accel_threshold) {
    return false;
  }

  return true;
}

void MotionDetector::setGyroThreshold(double threshold) { m_gyro_threshold = threshold; }

void MotionDetector::setAccelThreshold(double threshold) { m_accel_threshold = threshold; }

}  // namespace adi_imu
