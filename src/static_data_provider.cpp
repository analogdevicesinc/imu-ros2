/***************************************************************************//**
*   @file   static_data_provider.cpp
*   @brief  Implementation for static data
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

#include "imu_ros2/static_data_provider.h"

StaticDataProvider::StaticDataProvider()
{
    init();
}

StaticDataProvider::~StaticDataProvider()
{

}

void StaticDataProvider::init()
{
    // initialize a library
}

imu_ros2::msg::StaticData StaticDataProvider::getData(int count)
{
    (int)count;
    imu_ros2::msg::StaticData message;
    message.firmware_revision = "FIRMWARE_REVISION_1";
    message.firmware_date = "19-01-2023";
    message.product_id = "PRODUCT_ID_1";
    message.serial_number = "SERIAL_NUMBER_1";
    message.flash_memory_write_counter = 1;

    return message;
}
