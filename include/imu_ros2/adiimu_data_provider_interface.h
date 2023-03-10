/***************************************************************************//**
*   @file   adiimu_data_provider_interface.h
*   @brief  Interface for providing acceleration, gyroscope,
*           temperature, delta velocity and delta angle data
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

#ifndef ADIIMU_DATA_PROVIDER_INTERFACE_H
#define ADIIMU_DATA_PROVIDER_INTERFACE_H

#include <string>
#include "imu_ros2/msg/adi_imu_data.hpp"

class AdiImuDataProviderInterface {

public:
    AdiImuDataProviderInterface(){}
    virtual ~AdiImuDataProviderInterface(){}

    virtual void init() = 0;
    virtual imu_ros2::msg::AdiImuData getData(int count) = 0;
};

#endif // ADIIMU_DATA_PROVIDER_INTERFACE_H
