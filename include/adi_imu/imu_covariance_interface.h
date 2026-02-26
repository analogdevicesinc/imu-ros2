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

#ifndef ADI_IMU__IMU_COVARIANCE_INTERFACE_H_
#define ADI_IMU__IMU_COVARIANCE_INTERFACE_H_

#include <array>
#include <geometry_msgs/msg/vector3.hpp>

namespace adi_imu
{
/**
    * @brief 3x3 covariance matrix stored as row-major array
    * Layout: [xx ,xy, xz, yx, yy, yz, zx, zy, zz]
     */
using CovarianceMatrix = std::array<double, 9>;

/**
     * @brief 3D vector for acceleration or angular velocity samples using ROS2 built-in geometry_msgs::msg::Vector3 interface.
    */
using Vec3 = geometry_msgs::msg::Vector3;

/**
      * @brief Interface for IMU covariance computation strategies.
      *
      * Implementation can provide static covariance values, compute them 
      * online during a calibration phase, or adapt them continuously
       */

class ImuCovarianceInterface
{
public:
  ImuCovarianceInterface() = default;
  virtual ~ImuCovarianceInterface() = default;

  // Prevent copying
  ImuCovarianceInterface(const ImuCovarianceInterface &) = delete;
  ImuCovarianceInterface & operator=(const ImuCovarianceInterface &) = delete;

  /**
            * @brief Process a new IMU sample for covariance estimation.
            * @param accel Linear acceleration sample (m/s^2)
            * @param gyro Angular velocity sample (rad/s)
            */
  virtual void addSample(const Vec3 & accel, const Vec3 & gyro) = 0;

  /**
            * @brief Check if covariance estimation is ready (calibration complete).
            * @return true if covariance values are valid and ready to use.
             */
  virtual bool isReady() const = 0;

  /**
            * @brief Get the linear acceleration covariance matrix.
            * @return 3x3 covariance matrix row-major order.
            */
  virtual CovarianceMatrix getAccelCovariance() const = 0;

  /**
            * @brief Get the angular velocity covariance matrix.
            * @return 3x3 covariance matrix row-major order.
            */
  virtual CovarianceMatrix getGyroCovariance() const = 0;

  /**
            * @brief Reset the covariance extimator to initial state.
            */
  virtual void reset() = 0;

  /**
            * @brief Get the current calibration progress (0.0 to 1.0).
            * @return Progress ratio, 1.0 when calibration is complete.
             */
  virtual double getCalibrationProgress() const = 0;
};

}  // namespace adi_imu

#endif  // ADI_IMU__IMU_COVARIANCE_INTERFACE_H_