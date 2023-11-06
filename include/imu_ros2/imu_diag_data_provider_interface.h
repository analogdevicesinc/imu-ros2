/*******************************************************************************
 *   @file   imu_diag_data_provider_interface.h
 *   @brief  Interface for providing diagnosis data for adis.
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

#ifndef IMU_DIAG_DATA_PROVIDER_INTERFACE_H
#define IMU_DIAG_DATA_PROVIDER_INTERFACE_H

#include "imu_ros2/msg/imu_diag_data.hpp"

/**
 * @brief Interface for diagnosis data provider for adis chips.
 */
class ImuDiagDataProviderInterface
{
public:
  /**
   * @brief Constructor for ImuDiagDataProviderInterface.
   */
  ImuDiagDataProviderInterface() {}

  /**
   * @brief Destructor for ImuDiagDataProviderInterface.
   */
  virtual ~ImuDiagDataProviderInterface() {}

  /**
   * @brief Populate ImuDiagData message with diagnosis data.
   * @param message Message containing the diagnosis data.
   * @return Return true if the message parameter is successfully populated with
   * diagnosis data and false otherwise.
   */
  virtual bool getData(imu_ros2::msg::ImuDiagData & message) = 0;
};

#endif  // IMU_DIAG_DATA_PROVIDER_INTERFACE_H
