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

#ifndef ADI_IMU__STATIC_COVARIANCE_PROVIDER_H_
#define ADI_IMU__STATIC_COVARIANCE_PROVIDER_H_

#include "adi_imu/imu_covariance_interface.h"

namespace adi_imu
{

/**
    * @brief Provides fixed/static covariance values from configuration.
    * Use this when covariance values are known from datasheet specifications
    * or prior calibration. No runtime computatino is performed.
     */

class StaticCovarianceProvider : public ImuCovarianceInterface
{
public:
  // Default static covariance values
  inline static constexpr double DEFAULT_STATIC_ACCEL_COVARIANCE = 0.01;
  inline static constexpr double DEFAULT_STATIC_GYRO_COVARIANCE = 0.001;

  /**
        * @brief Construct diagonal covariance values. Measurements are not intercorrelated.
        * @param accel_variance Variance for each accelerometer axis (m/s^2)^2
        * @param gyro_variance Variance for each gyroscope axis (rad/s)^2.
         */
  StaticCovarianceProvider(const Vec3 & accel_variance, const Vec3 & gyro_variance);

  /**
        * @brief Construct with full covariance matrices.
        * @param accel_cov Full 3x3 accelerometer covariance matrix
        * @param gyro_cov Full 3x3 gyroscope covariance matrix
         */
  StaticCovarianceProvider(const CovarianceMatrix & accel_cov, const CovarianceMatrix & gyro_cov);

  void addSample(const Vec3 & accel, const Vec3 & gyro) override;
  bool isReady() const override;
  CovarianceMatrix getAccelCovariance() const override;
  CovarianceMatrix getGyroCovariance() const override;
  void reset() override;
  double getCalibrationProgress() const override;

private:
  CovarianceMatrix m_accel_covariance;
  CovarianceMatrix m_gyro_covariance;
};

}  // namespace adi_imu

#endif  // ADI_IMU__STATIC_COVARIANCE_PROVIDER_H_
