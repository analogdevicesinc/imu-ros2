/*******************************************************************************
 *   @file   imu_1650x_diag_data_provider_interface.h
 *   @brief  Interface for adis1650x diagnosis publisher.
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

#ifndef IMU_1650X_DIAG_DATA_PROVIDER_INTERFACE_H
#define IMU_1650X_DIAG_DATA_PROVIDER_INTERFACE_H

#include "imu_ros2/msg/imu1650x_diag_data.hpp"


/**
 * \brief Interface for diagnosis data provider.
 *
 * This interface provides data for the publisher.
 * The type of data is Imu16505DiagData.
 */
class Imu1650xDiagDataProviderInterface
{
public:
  /**
   * \brief Constructor for Imu1650xDiagDataProviderInterface.
   *
   * This is the default constructor for interface
   *  Imu1650xDiagDataProviderInterface.
   *
   */
  Imu1650xDiagDataProviderInterface() {}

  /**
   * \brief Destructor for Imu1650xDiagDataProviderInterface.
   *
   * This is a virtual destructor for Imu1650xDiagDataProviderInterface.
   *
   */
  virtual ~Imu1650xDiagDataProviderInterface() {}

  /**
   * @brief Populate message variable with data.
   *
   * This function return by parameter a message variable
   * with data from the sensor like diag_data_path_overrun.
   *
   * @return Return true if the message variable is populated with
   *  values and false if the message is not populated.
   * @param message Populate message variable
   * with data like diag_data_path_overrun, diag_flash_memory_update_error,
   * diag_spi_communication_error, diag_standby_mode etc.
   */
  virtual bool getData(imu_ros2::msg::Imu1650xDiagData & message) = 0;
};

#endif  // IMU_1650X_DIAG_DATA_PROVIDER_INTERFACE_H
