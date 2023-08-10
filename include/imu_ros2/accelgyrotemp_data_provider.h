/*******************************************************************************
 *   @file   accelgyrotemp_data_provider.h
 *   @brief  Header for providing acceleration, gyroscope and
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

#ifndef ACCELGYROTEMP_DATA_PROVIDER_H
#define ACCELGYROTEMP_DATA_PROVIDER_H

#include "imu_ros2/accelgyrotemp_data_provider_interface.h"
#include "imu_ros2/iio_wrapper.h"

/**
 * @brief Class for acceleration, angular velocity and temperature
 * buffered data provider.
 */
class AccelGyroTempDataProvider : public AccelGyroTempDataProviderInterface
{
public:
  /**
   * @brief Constructor for AccelGyroTempDataProvider.
   */
  AccelGyroTempDataProvider();

  /**
   * @brief Destructor for AccelGyroTempDataProvider.
   */
  ~AccelGyroTempDataProvider();

  /**
   * @brief Populate AccelGyroTempData message with measured data.
   * @param message Message containing the measured data.
   * @return Return true if the message parameter is successfully populated with
   * measured data and false otherwise.
   */
  bool getData(imu_ros2::msg::AccelGyroTempData & message) override;

private:
  /*! This data member is used to access sensor information via libiio. */
  IIOWrapper m_iio_wrapper;
};

#endif  // ACCELGYROTEMP_DATA_PROVIDER_H
