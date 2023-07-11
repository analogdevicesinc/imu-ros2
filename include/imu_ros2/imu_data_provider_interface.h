/*******************************************************************************
 *   @file   imu_ros_publisher_interface.h
 *   @brief  Interface for providing IMU ros standard data.
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

#ifndef IMU_DATA_PROVIDER_INTERFACE_H
#define IMU_DATA_PROVIDER_INTERFACE_H

#include <sensor_msgs/msg/imu.hpp>

/**
 * \brief Interface for accel gyro data provider.
 *
 * This interface provides data for the publisher.
 * The type of data is the standard sensor_msgs::msg::Imu.
 */
class ImuDataProviderInterface
{
public:
  /**
   * \brief Constructor for ImuDataProviderInterface.
   *
   * This is the default constructor for interface
   *  ImuDataProviderInterface.
   *
   */
  ImuDataProviderInterface() {}

  /**
   * \brief Destructor for ImuDataProviderInterface.
   *
   * This is a virtual destructor for ImuDataProviderInterface.
   *
   */
  virtual ~ImuDataProviderInterface() {}

  /**
   * \brief Enable buffer data reading.
   *
   * This function enable buffer variable from libiio
   * to read very fast.
   *
   * \return Return true if the buffer was enabled and
   * false if the buffer was not enabled.
   */
  virtual bool enableBufferedDataOutput() = 0;

  /**
   * @brief Populate message variable with data.
   *
   * This function return by parameter a message variable
   * with data from the sensor like acceleration and gyroscope.
   *  The reading from the libiio is with buffer.
   *
   * @return Return true if the message variable is populated with
   *  values and false if the message is not populated.
   * @param message Populate message variable
   * with data like acceleration and gyroscope from
   * the sensor.
   */
  virtual bool getData(sensor_msgs::msg::Imu & message) = 0;
};

#endif  // IMU_DATA_PROVIDER_INTERFACE_H
