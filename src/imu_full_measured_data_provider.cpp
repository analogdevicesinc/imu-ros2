/***************************************************************************//**
*   @file   imu_full_measured_data_provider.cpp
*   @brief  Implementation for imu full measured data
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

#include "imu_ros2/imu_full_measured_data_provider.h"

ImuFullMeasuredDataProvider::ImuFullMeasuredDataProvider()
{
    init();
}

ImuFullMeasuredDataProvider::~ImuFullMeasuredDataProvider()
{

}

void ImuFullMeasuredDataProvider::init()
{
    // initialize a library
}

bool ImuFullMeasuredDataProvider::getData(imu_ros2::msg::ImuFullMeasuredData& data)
{
    bool success;

    success = m_iioWrapper.getRegAccelerometerX(data.accel.x);
    if(!success)
        return false;

    success = m_iioWrapper.getRegAccelerometerY(data.accel.y);
    if(!success)
        return false;

    success = m_iioWrapper.getRegAccelerometerZ(data.accel.z);
    if(!success)
        return false;

    success = m_iioWrapper.getRegGyroscopeX(data.gyro.x);
    if(!success)
        return false;

    success = m_iioWrapper.getRegGyroscopeY(data.gyro.y);
    if(!success)
        return false;

    success = m_iioWrapper.getRegGyroscopeZ(data.gyro.z);
    if(!success)
        return false;

    success = m_iioWrapper.getRegVelocityX(data.delta_vel.x);
    if(!success)
        return false;

    success = m_iioWrapper.getRegVelocityY(data.delta_vel.y);
    if(!success)
        return false;

    success = m_iioWrapper.getRegVelocityZ(data.delta_vel.z);
    if(!success)
        return false;

    success = m_iioWrapper.getRegRotX(data.delta_angle.x);
    if(!success)
        return false;

    success = m_iioWrapper.getRegRotY(data.delta_angle.y);
    if(!success)
        return false;

    success = m_iioWrapper.getRegRotZ(data.delta_angle.z);
    if(!success)
        return false;

    success = m_iioWrapper.getRegTemperature(data.temp);

    return success;
}
