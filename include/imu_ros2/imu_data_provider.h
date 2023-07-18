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

#include "imu_ros2/iio_wrapper.h"
#include "imu_ros2/imu_data_provider_interface.h"

/**
 * \brief Class implementation for accel and gyro data provider.
 *
 * This class implements  ImuDataProviderInterface interface.
 * This class provides data for the publisher.
 * The type of data is the standard sensor_msgs::msg::Imu.
 */
class ImuDataProvider : public ImuDataProviderInterface
{
public:
  /**
   * \brief Constructor for ImuDataProvider.
   *
   * This is the default constructor for class
   *  ImuDataProvider.
   *
   */
  ImuDataProvider();

  /**
   * \brief Destructor for ImuDataProvider.
   *
   * This is the destructor for ImuDataProvider.
   *
   */
  ~ImuDataProvider();

  /**
   * \brief Enable buffer data reading.
   *
   * This function enable buffer variable from libiio
   * to read very fast.
   *
   * \return Return true if the buffer was enabled and
   * false if the buffer was not enabled.
   */
  bool enableBufferedDataOutput() override;

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
  bool getData(sensor_msgs::msg::Imu & message) override;

private:
  /*! This data member access information from libiio */
  IIOWrapper m_iio_wrapper;
};

#endif  // IMU_DATA_PROVIDER_H
