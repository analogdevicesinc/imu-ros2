/***************************************************************************//**
*   @file   acceleration_data_provider.cpp
*   @brief  Implementation for acceleration
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

#include "imu_ros2/acceleration_data_provider.h"
#include <sensor_msgs/msg/imu.hpp>

AccelerationDataProvider::AccelerationDataProvider()
{
    init();
}

AccelerationDataProvider::~AccelerationDataProvider()
{

}

void AccelerationDataProvider::init()
{
    // initialize a library
}

void AccelerationDataProvider::load()
{
    m_iioWrapper.load();
}

void AccelerationDataProvider::unload()
{
    m_iioWrapper.unload();
}

sensor_msgs::msg::Imu AccelerationDataProvider::getData(bool& success)
{
    sensor_msgs::msg::Imu message;

    m_iioWrapper.update_buffer(success);

    if(m_iioWrapper.isBufferLoaded() == false)
        return message;

    if(success)
    {
        message.linear_acceleration.x = m_iioWrapper.getAccelerometerX();
        message.linear_acceleration.y = m_iioWrapper.getAccelerometerY();
        message.linear_acceleration.z = m_iioWrapper.getAccelerometerZ();

        message.angular_velocity.x = m_iioWrapper.getGyroscopeX();
        message.angular_velocity.y = m_iioWrapper.getGyroscopeY();
        message.angular_velocity.z = m_iioWrapper.getGyroscopeZ();
    }

    return message;
}
