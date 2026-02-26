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

#include "adi_imu/static_covariance_provider.h"

namespace adi_imu
{
StaticCovarianceProvider::StaticCovarianceProvider(
  const Vec3 & accel_variance, const Vec3 & gyro_variance)
{
  // Create diagonal covariance matrices
  m_accel_covariance = {accel_variance.x, 0.0, 0.0, 0.0, accel_variance.y, 0.0, 0.0, 0.0,
                        accel_variance.z};
  m_gyro_covariance = {gyro_variance.x, 0.0, 0.0, 0.0, gyro_variance.y, 0.0, 0.0, 0.0,
                       gyro_variance.z};
}

StaticCovarianceProvider::StaticCovarianceProvider(
  const CovarianceMatrix & accel_cov, const CovarianceMatrix & gyro_cov)
: m_accel_covariance(accel_cov), m_gyro_covariance(gyro_cov)
{
}

void StaticCovarianceProvider::addSample(const Vec3 &, const Vec3 &)
{
  // Nothing to add here for the static covariances
}

bool StaticCovarianceProvider::isReady() const { return true; }

CovarianceMatrix StaticCovarianceProvider::getAccelCovariance() const { return m_accel_covariance; }

CovarianceMatrix StaticCovarianceProvider::getGyroCovariance() const { return m_gyro_covariance; }

void StaticCovarianceProvider::reset()
{
  // No-op for static provider
}

double StaticCovarianceProvider::getCalibrationProgress() const { return 1.0; }
}  // namespace adi_imu