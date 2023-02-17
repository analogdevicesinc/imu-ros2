/***************************************************************************//**
*   @file   static_data_provider.h
*   @brief  Provide static data like product id
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

#ifndef STATIC_DATA_PROVIDER_H
#define STATIC_DATA_PROVIDER_H

#include "imu_ros2/static_data_provider_interface.h"

class StaticDataProvider : public StaticDataProviderInterface {


public:
    StaticDataProvider();
    ~StaticDataProvider();

    void init() override;
    imu_ros2::msg::StaticData getData(int count) override;
};

#endif // STATIC_DATA_PROVIDER_STRING_H
