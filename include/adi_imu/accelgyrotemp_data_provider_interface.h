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

#include "adi_imu/msg/accel_gyro_temp_data.hpp"

/**
 * @brief Interface for acceleration, angular velocity and temperature
 * buffered data provider.
 */
class AccelGyroTempDataProviderInterface
{
public:
  /**
   * @brief Constructor for AccelGyroTempDataProviderInterface.
   */
  AccelGyroTempDataProviderInterface() {}

  /**
   * @brief Destructor for AccelGyroTempDataProviderInterface.
   */
  virtual ~AccelGyroTempDataProviderInterface() {}

  /**
   * @brief Populate AccelGyroTempData message with measured data.
   * @param message Message containing the measured data.
   * @return Return true if the message parameter is successfully populated with
   * measured data and false otherwise.
   */
  virtual bool getData(adi_imu::msg::AccelGyroTempData & message) = 0;
};

#endif  // ACCELGYROTEMP_DATA_PROVIDER_INTERFACE_H
