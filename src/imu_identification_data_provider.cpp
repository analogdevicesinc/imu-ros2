/***************************************************************************//**
*   @file   imu_identification_data_provider.cpp
*   @brief  Implementation for imu identification data
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

#include "imu_ros2/imu_identification_data_provider.h"

ImuIdentificationDataProvider::ImuIdentificationDataProvider()
{
    init();
}

ImuIdentificationDataProvider::~ImuIdentificationDataProvider()
{

}

void ImuIdentificationDataProvider::init()
{
    // initialize a library
}

imu_ros2::msg::ImuIdentificationData ImuIdentificationDataProvider::getData()
{
    imu_ros2::msg::ImuIdentificationData message;

    if(m_iioWrapper.isBufferLoaded() == false)
        return message;

    message.firmware_revision = m_iioWrapper.firmware_revision();
    message.firmware_date = m_iioWrapper.firmware_date();
    message.product_id = m_iioWrapper.product_id();
    message.serial_number = m_iioWrapper.serial_number();
    message.gyroscope_measurement_range = m_iioWrapper.gyroscope_measurement_range();

    return message;
}
