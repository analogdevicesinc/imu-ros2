/***************************************************************************//**
*   @file   static_data_provider_interface.h
*   @brief  Interface for providing static data
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

#ifndef STATIC_DATA_PROVIDER_INTERFACE_H
#define STATIC_DATA_PROVIDER_INTERFACE_H

#include <string>

//firmware revision, firmware date, product id, serial number, flash memory write counter
//struct StaticData {
//    std::string m_firmwareRevision;
//    std::string m_firmwareDate;
//    std::string m_productId;
//    std::string m_serialNumber;
//    int32_t m_flashMemoryWriteCounter;
//};

#include "imu_ros2/msg/static_data.hpp"

class StaticDataProviderInterface {

public:
    StaticDataProviderInterface(){}
    virtual ~StaticDataProviderInterface(){}

    virtual void init() = 0;
    virtual imu_ros2::msg::StaticData getData(int count) = 0;
};

#endif // STATICN_DATA_PROVIDER_INTERFACE_H
