/***************************************************************************//**
*   @file   gyroscope_data_provider.cpp
*   @brief  Implementation for gyroscope data
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

#include "imu_ros2/gyroscope_data_provider.h"

GyroscopeDataProvider::GyroscopeDataProvider()
{
    init();
}

GyroscopeDataProvider::~GyroscopeDataProvider()
{

}

void GyroscopeDataProvider::init()
{
    // initialize a library
}

imu_ros2::msg::GyroscopeData GyroscopeDataProvider::getData(int count)
{
    (int)count;
    imu_ros2::msg::GyroscopeData message;
    message.anglvel_x =m_iioWrapper.getGyroscopeX();
    message.anglvel_y =m_iioWrapper.getGyroscopeY();
    message.anglvel_z =m_iioWrapper.getGyroscopeZ();


    return message;
}
