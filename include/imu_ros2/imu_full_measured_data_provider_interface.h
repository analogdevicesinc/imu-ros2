/***************************************************************************//**
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

class ImuFullMeasuredDataProviderInterface
{

public:
  ImuFullMeasuredDataProviderInterface() {}
  virtual ~ImuFullMeasuredDataProviderInterface() {}

  virtual bool getData(imu_ros2::msg::ImuFullMeasuredData& data) = 0;
};

#endif // IMU_FULL_MEASURED_DATA_PROVIDER_INTERFACE_H
