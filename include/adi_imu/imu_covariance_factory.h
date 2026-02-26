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

#ifndef ADI_IMU__IMU_COVARIANCE_FACTORY_H_
#define ADI_IMU__IMU_COVARIANCE_FACTORY_H_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "adi_imu/imu_covariance_interface.h"

namespace adi_imu
{
/**
    * @brief Covariance computation algorithm types.
     */

enum class CovarianceAlgorithm
{
  STATIC,          // Fixed values from parameters
  WELFORD_ONLINE,  // Calibration-based (Welford's algorithm)
  SLIDING_WINDOW,  // Adaptive sliding window
  EWMA,            // Exponentially-Weighted Moving Average (EWMA)
  KALMAN,          // Kalman filter-based variance estimation
};

/**
     * @brief Factory for creating covariance providers based on configuration.
      */
class ImuCovarianceFactory
{
public:
  /**
            * @brief Create a covariance provider from ROS2 parameters.
            * @param node ROS2 node for parameter access.
            * @return Unique pointer to covariance provider, or nullptr if disabled
             */
  static std::unique_ptr<ImuCovarianceInterface> createFromParameters(
    const std::shared_ptr<rclcpp::Node> & node);

  /**
            * @brief Create a covariance provider by algorithm type.
            * @param params Algorithm-specific parameters.
            * @return Unique pointer to covariance provider.
             */
  static std::unique_ptr<ImuCovarianceInterface> create(
    CovarianceAlgorithm algorithm, const std::shared_ptr<rclcpp::Node> & node);

  /**
            * @brief Parse algorithm type from string
            * @param algorithm_str String representation ("static, welford", "sliding_window", "ewma", "kalman").
            * @return Corresponding enum value.
             */
  static CovarianceAlgorithm parseAlgorithm(const std::string & algorithm_str);
};
}  // namespace adi_imu

#endif  // ADI_IMU__IMU_COVARIANCE_FACTORY_H_
