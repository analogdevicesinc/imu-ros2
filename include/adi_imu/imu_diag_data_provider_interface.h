/*******************************************************************************
 *   @file   imu_diag_data_provider_interface.h
 *   @brief  Interface for providing diagnosis data for adis.
 *   @author Vasile Holonec (Vasile.Holonec@analog.com)
*******************************************************************************/
// Copyright 2023 Analog Devices, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ADI_IMU__IMU_DIAG_DATA_PROVIDER_INTERFACE_H_
#define ADI_IMU__IMU_DIAG_DATA_PROVIDER_INTERFACE_H_

#include "adi_imu/adis_register_map.h"
#include "adi_imu/msg/imu_diag_data_adis1646_x.hpp"
#include "adi_imu/msg/imu_diag_data_adis1647_x.hpp"
#include "adi_imu/msg/imu_diag_data_adis1650_x.hpp"
#include "adi_imu/msg/imu_diag_data_adis1654_x.hpp"
#include "adi_imu/msg/imu_diag_data_adis1655_x.hpp"
#include "adi_imu/msg/imu_diag_data_adis1657_x.hpp"

namespace adi_imu
{
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
  virtual bool getData(adi_imu::msg::ImuDiagDataADIS1646X & message) = 0;
  virtual bool getData(adi_imu::msg::ImuDiagDataADIS1647X & message) = 0;
  virtual bool getData(adi_imu::msg::ImuDiagDataADIS1650X & message) = 0;
  virtual bool getData(adi_imu::msg::ImuDiagDataADIS1654X & message) = 0;
  virtual bool getData(adi_imu::msg::ImuDiagDataADIS1655X & message) = 0;
  virtual bool getData(adi_imu::msg::ImuDiagDataADIS1657X & message) = 0;
};

}  // namespace adi_imu

#endif  // ADI_IMU__IMU_DIAG_DATA_PROVIDER_INTERFACE_H_
