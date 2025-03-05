/*******************************************************************************
 *   @file   imu_ros_publisher.h
 *   @brief  Header for providing IMU ros standard data.
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

#ifndef IMU_DATA_PROVIDER_H
#define IMU_DATA_PROVIDER_H

#include "adi_imu/iio_wrapper.h"
#include "adi_imu/imu_data_provider_interface.h"

/**
 * @brief Class for standard message sensor_msgs::msg::Imu data provider.
 */
class ImuDataProvider : public ImuDataProviderInterface
{
public:
  /**
   * @brief Constructor for ImuDataProvider.
   */
  ImuDataProvider();

  /**
   * @brief Destructor for ImuDataProvider.
   */
  ~ImuDataProvider();

  /**
   * @brief Populate Imu message with measured data.
   * @param message Message containing the measured data.
   * @return Return true if the message parameter is successfully populated with
   * measured data and false otherwise.
   */
  bool getData(sensor_msgs::msg::Imu & message) override;

private:
  /*! This data member is used to access sensor information via libiio. */
  IIOWrapper m_iio_wrapper;
};

#endif  // IMU_DATA_PROVIDER_H
