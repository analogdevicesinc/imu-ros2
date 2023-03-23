/***************************************************************************//**
*   @file   imu_control_data_provider.cpp
*   @brief  Implementation for imu control data
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

#include "imu_ros2/imu_control_data_provider.h"

ImuControlDataProvider::ImuControlDataProvider()
{
    init();
}

ImuControlDataProvider::~ImuControlDataProvider()
{

}

void ImuControlDataProvider::init()
{
    // initialize a library
    m_filter_size = 0;
}

void ImuControlDataProvider::set_filter_size(int32_t val)
{
    if(val != m_filter_size)
    {
        m_iioWrapper.set_filter_size(val);
        m_filter_size = val;
    }
}

imu_ros2::msg::ImuControlData ImuControlDataProvider::getData()
{
    imu_ros2::msg::ImuControlData message;
    message.filter_size = m_iioWrapper.filter_size();

    return message;
}
