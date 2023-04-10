/***************************************************************************//**
*   @file   adiimu_data_provider.cpp
*   @brief  Implementation for adi imu data
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

#include "imu_ros2/adiimu_data_provider.h"

AdiImuDataProvider::AdiImuDataProvider()
{
    init();
}

AdiImuDataProvider::~AdiImuDataProvider()
{

}

void AdiImuDataProvider::init()
{
    // initialize a library
}

imu_ros2::msg::AdiImuData AdiImuDataProvider::getData(bool& success)
{
    imu_ros2::msg::AdiImuData message;

    m_iioWrapper.update_buffer(success);

    if(success)
    {
        message.accel.x = m_iioWrapper.getAccelerometerX();
        message.accel.y = m_iioWrapper.getAccelerometerY();
        message.accel.z = m_iioWrapper.getAccelerometerZ();

        message.gyro.x = m_iioWrapper.getGyroscopeX();
        message.gyro.y = m_iioWrapper.getGyroscopeY();
        message.gyro.z = m_iioWrapper.getGyroscopeZ();

        message.delta_vel.x = m_iioWrapper.getVelocityX();
        message.delta_vel.y = m_iioWrapper.getVelocityY();
        message.delta_vel.z = m_iioWrapper.getVelocityZ();

        message.delta_angle.x = m_iioWrapper.getRotX();
        message.delta_angle.y = m_iioWrapper.getRotY();
        message.delta_angle.z = m_iioWrapper.getRotZ();

        message.temp = m_iioWrapper.getTemperature();

        message.count = m_iioWrapper.count();
    }

    return message;
}
