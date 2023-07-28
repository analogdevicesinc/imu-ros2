/*******************************************************************************
 *   @file   imu_full_measured_data_provider.h
 *   @brief  Header for providing acceleration, gyroscope, temperature,
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

#ifndef IMU_FULL_MEASURED_DATA_PROVIDER_H
#define IMU_FULL_MEASURED_DATA_PROVIDER_H

#include "imu_ros2/iio_wrapper.h"
#include "imu_ros2/imu_full_measured_data_provider_interface.h"

/**
 * \brief Class implementation for accel, gyro, temp, delta velocity,
 *  delta angle data provider.
 *
 * This class implements  ImuFullMeasuredDataProviderInterface interface.
 * This class provide data for the publisher.
 * The type of data is ImuFullMeasuredData.
 */
class ImuFullMeasuredDataProvider : public ImuFullMeasuredDataProviderInterface
{
public:
  /**
   * \brief Constructor for ImuFullMeasuredDataProvider.
   *
   * This is the default constructor for class
   *  ImuFullMeasuredDataProvider.
   *
   */
  ImuFullMeasuredDataProvider();

  /**
   * \brief Destructor for ImuFullMeasuredDataProvider.
   *
   * This is the destructor for ImuFullMeasuredDataProvider.
   *
   */
  ~ImuFullMeasuredDataProvider();

  /**
   * \brief Configures the type of data to be written in the buffer
   *
   * This function configures the type of data to be written in the buffer.
   *
   * \return Return true if the configures is successful, false otherwise.
   */
  bool configureBufferedDataOutput() override;

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
  bool getData(imu_ros2::msg::ImuFullMeasuredData & data) override;

private:
  /*! This data member access information from libiio */
  IIOWrapper m_iio_wrapper;
};

#endif  // IMU_FULL_MEASURED_DATA_PROVIDER_H
