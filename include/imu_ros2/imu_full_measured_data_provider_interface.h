/*******************************************************************************
 *   @file   imu_full_measured_data_provider_interface.h
 *   @brief  Interface for providing acceleration, gyroscope, temperature,
 *           delta velocity, delta angle and temperature data.
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

#ifndef IMU_FULL_MEASURED_DATA_PROVIDER_INTERFACE_H
#define IMU_FULL_MEASURED_DATA_PROVIDER_INTERFACE_H

#include "imu_ros2/msg/imu_full_measured_data.hpp"

/**
 * \brief Interface for full measured data like
 * accel, gyro, temp, delta velocity, delta angle.
 *
 * This interface provides data for the publisher.
 * The type of data is ImuFullMeasuredData.
 */
class ImuFullMeasuredDataProviderInterface
{
public:
  /**
   * \brief Constructor for ImuFullMeasuredDataProviderInterface.
   *
   * This is the default constructor for interface
   *  ImuFullMeasuredDataProviderInterface.
   *
   */
  ImuFullMeasuredDataProviderInterface() {}

  /**
   * \brief Destructor for ImuFullMeasuredDataProviderInterface.
   *
   * This is a virtual destructor for ImuFullMeasuredDataProviderInterface.
   *
   */
  virtual ~ImuFullMeasuredDataProviderInterface() {}

  /**
   * @brief Populate message variable with data.
   *
   * This function return by parameter a message variable
   * with data from the sensor like acceleration, gyroscope
   * temperature, delta velocity and delta angle.
   *  The reading from the libiio is with scale factor.
   *
   * @return Return true if the message variable is populated with
   *  values and false if the message is not populated.
   * @param data Populate data variable with data like
   * acceleration, gyroscope, temperature, delta velocity,
   * delta angle from the sensor.
   */
  virtual bool getData(imu_ros2::msg::ImuFullMeasuredData & data) = 0;

  /**
   * \brief Configures the type of data to be written in the buffer
   *
   * This function configures the type of data to be written in the buffer.
   *
   * \return Return true if the configures is successful, false otherwise.
   */
  virtual bool configureBufferedDataOutput() = 0;
};

#endif  // IMU_FULL_MEASURED_DATA_PROVIDER_INTERFACE_H
