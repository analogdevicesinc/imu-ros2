/*******************************************************************************
 *   @file   accelgyrotemp_data_provider_interface.h
 *   @brief  Interface for providing acceleration, gyroscope and
 *           temperature data provider.
 *   @author Vasile Holonec (Vasile.Holonec@analog.com)
 *******************************************************************************
 * Copyright 2023(c) Analog Devices, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ******************************************************************************/

#ifndef ACCELGYROTEMP_DATA_PROVIDER_INTERFACE_H
#define ACCELGYROTEMP_DATA_PROVIDER_INTERFACE_H

#include "imu_ros2/msg/accel_gyro_temp_data.hpp"

/**
 * \brief Interface for accel gyro temp data provider.
 *
 * This interface provides data for the publisher.
 * The type of data is AccelGyroTempData.
 */
class AccelGyroTempDataProviderInterface
{
public:
  /**
   * \brief Constructor for AccelGyroTempDataProviderInterface.
   *
   * This is the default constructor for interface
   *  AccelGyroTempDataProviderInterface.
   *
   */
  AccelGyroTempDataProviderInterface() {}

  /**
   * \brief Destructor for AccelGyroTempDataProviderInterface.
   *
   * This is a virtual destructor for AccelGyroTempDataProviderInterface.
   *
   */
  virtual ~AccelGyroTempDataProviderInterface() {}

  /**
   * @brief Populate message variable with data.
   *
   * This function return by parameter a message variable
   * with data from the sensor like acceleration, gyroscope
   * and temperature. The reading from the libiio is with buffer.
   *
   * @return Return true if the message variable is populated with
   *  values and false if the message is not populated.
   * @param message Populate message variable
   * with data like acceleration, gyroscope and temperature from
   * the sensor.
   */
  virtual bool getData(imu_ros2::msg::AccelGyroTempData & message) = 0;
};

#endif  // ACCELGYROTEMP_DATA_PROVIDER_INTERFACE_H
