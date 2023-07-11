/*******************************************************************************
 *   @file   imu_1657x_diag_data_provider.h
 *   @brief  Header for providing diagnosis data for adis1657x.
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

#ifndef IMU_1657X_DIAG_DATA_PROVIDER_H
#define IMU_1657X_DIAG_DATA_PROVIDER_H

#include "imu_ros2/iio_wrapper.h"
#include "imu_ros2/imu_1657x_diag_data_provider_interface.h"

/**
 * \brief Class implementation for diag data provider.
 *
 * This class implements Imu1657xDiagDataProviderInterface interface.
 * This class provide data for the publisher.
 * The type of data is Imu1657xDiagData.
 */
class Imu1657xDiagDataProvider : public Imu1657xDiagDataProviderInterface
{
public:
  /**
   * \brief Constructor for Imu1657xDiagDataProvider.
   *
   * This is the default constructor for class
   *  Imu1657xDiagDataProvider.
   *
   */
  Imu1657xDiagDataProvider();

  /**
   * \brief Destructor for Imu1657xDiagDataProvider.
   *
   * This is the destructor for Imu1657xDiagDataProvider.
   *
   */
  ~Imu1657xDiagDataProvider();

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
  bool getData(imu_ros2::msg::Imu1657xDiagData & message) override;

private:
  IIOWrapper m_iio_wrapper; /**< This data member access information from libiio */
};

#endif  // IMU_1657X_DIAG_DATA_PROVIDER_H
