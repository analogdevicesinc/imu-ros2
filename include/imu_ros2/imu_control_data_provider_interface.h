/***************************************************************************//**
*   @file   imu_control_data_provider_interface.h
*   @brief  Interface for providing imu control data
*   @author Vasile Holonec (Vasile.Holonec@analog.com)
********************************************************************************
* Copyright 2023(c) Analog Devices, Inc.

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

#ifndef IMU_CONTROL_DATA_PROVIDER_INTERFACE_H
#define IMU_CONTROL_DATA_PROVIDER_INTERFACE_H

#include <string>
#include "imu_ros2/msg/imu_control_data.hpp"

class ImuControlDataProviderInterface {

public:
    ImuControlDataProviderInterface(){}
    virtual ~ImuControlDataProviderInterface(){}

    virtual void init() = 0;
    virtual imu_ros2::msg::ImuControlData getData() = 0;

    virtual void set_filter_size(int32_t val) = 0;
};

#endif // IMU_CONTROL_DATA_PROVIDER_INTERFACE_H
