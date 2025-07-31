/*******************************************************************************
 *   @file   imu_full_measured_data_provider_interface.h
 *   @brief  Interface for providing acceleration, gyroscope, temperature,
 *           delta velocity, delta angle and temperature data.
 *   @author Vasile Holonec (Vasile.Holonec@analog.com)
*******************************************************************************/
// Copyright 2023 Analog Devices, Inc.
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

#ifndef ADI_IMU__IMU_FULL_MEASURED_DATA_PROVIDER_INTERFACE_H_
#define ADI_IMU__IMU_FULL_MEASURED_DATA_PROVIDER_INTERFACE_H_

#include "adi_imu/msg/imu_full_measured_data.hpp"

namespace adi_imu
{
/**
 * @brief Interface for acceleration, angular velocity, delta angle, delta
 * velocity and temperature data provider.
 */
class ImuFullMeasuredDataProviderInterface
{
public:
  /**
   * @brief Constructor for ImuFullMeasuredDataProviderInterface.
   */
  ImuFullMeasuredDataProviderInterface() {}

  /**
   * @brief Destructor for ImuFullMeasuredDataProviderInterface.
   */
  virtual ~ImuFullMeasuredDataProviderInterface() {}

  /**
   * @brief Populate ImuFullMeasuredData message with measured data.
   * @param message Message containing the measured data.
   * @return Return true if the message parameter is successfully populated with
   * measured data and false otherwise.
   */
  virtual bool getData(adi_imu::msg::ImuFullMeasuredData & message) = 0;
};

}  // namespace adi_imu

#endif  // ADI_IMU__IMU_FULL_MEASURED_DATA_PROVIDER_INTERFACE_H_
