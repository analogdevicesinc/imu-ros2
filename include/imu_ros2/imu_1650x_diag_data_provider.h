/*******************************************************************************
 *   @file   imu_1650x_diag_data_provider.h
 *   @brief  Header for providing diagnosis data for adis1650x.
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

#ifndef IMU_1650X_DIAG_DATA_PROVIDER_H
#define IMU_1650X_DIAG_DATA_PROVIDER_H

#include "imu_ros2/iio_wrapper.h"
#include "imu_ros2/imu_1650x_diag_data_provider_interface.h"

/**
 * @brief Class for diagnosis data provider for adis1650x chips.
 */
class Imu1650xDiagDataProvider : public Imu1650xDiagDataProviderInterface
{
public:
  /**
   * @brief Constructor for Imu1650xDiagDataProvider.
   */
  Imu1650xDiagDataProvider();

  /**
   * @brief Destructor for Imu1650xDiagDataProvider.
   */
  ~Imu1650xDiagDataProvider();

  /**
   * @brief Populate Imu1650xDiagData message with diagnosis data.
   * @param message Message containing the diagnosis data.
   * @return Return true if the message parameter is successfully populated with
   * diagnosis data and false otherwise.
   */
  bool getData(imu_ros2::msg::Imu1650xDiagData & message) override;

private:
  /*! This data member is used to access sensor information via libiio. */
  IIOWrapper m_iio_wrapper;
};

#endif  // IMU_1650X_DIAG_DATA_PROVIDER_H
