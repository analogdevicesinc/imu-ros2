/***************************************************************************/ /**
 *   @file   imu_identification_data_provider.h
 *   @brief  Header for providing imu identification data.
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
 *******************************************************************************/

#ifndef IMU_IDENTIFICATION_DATA_PROVIDER_H
#define IMU_IDENTIFICATION_DATA_PROVIDER_H

#include "imu_ros2/iio_wrapper.h"
#include "imu_ros2/imu_identification_data_provider_interface.h"

/**
 * \brief Class implementation for product id, serial number data provider.
 *
 * This class implements  ImuIdentificationDataProviderInterface interface.
 * This class provide data for the publisher.
 * The type of data is ImuIdentificationData.
 */
class ImuIdentificationDataProvider : public ImuIdentificationDataProviderInterface
{
public:
  /**
   * \brief Constructor for ImuIdentificationDataProvider.
   *
   * This is the default constructor for class
   *  ImuIdentificationDataProvider.
   *
   */
  ImuIdentificationDataProvider();

  /**
   * \brief Destructor for ImuIdentificationDataProvider.
   *
   * This is the destructor for ImuIdentificationDataProvider.
   *
   */
  ~ImuIdentificationDataProvider();

  /**
   * @brief Populate message variable with data.
   *
   * This function return by parameter a message variable
   * with data like product id, serial number.
   *
   * @return Return true if the message variable is populated with
   *  values and false if the message is not populated.
   * @param message Populate message variable
   * with data like product id, serial number.
   */
  bool getData(imu_ros2::msg::ImuIdentificationData & message) override;

private:
  /*! This data member access information from libiio */
  IIOWrapper m_iio_wrapper;
};

#endif  // IMU_IDENTIFICATION_DATA_PROVIDER_STRING_H
