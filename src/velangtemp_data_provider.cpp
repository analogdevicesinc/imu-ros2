/***************************************************************************//**
*   @file   velangtemp_data_provider.cpp
*   @brief  Implementation for vel ang temp data
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

#include "imu_ros2/velangtemp_data_provider.h"

VelAngTempDataProvider::VelAngTempDataProvider()
{
    init();
}

VelAngTempDataProvider::~VelAngTempDataProvider()
{

}

void VelAngTempDataProvider::init()
{
    // initialize a library
}

bool VelAngTempDataProvider::enableBufferedDataOutput()
{
    return (m_iioWrapper.update_burst_data_selection(1) == 0);
}

bool VelAngTempDataProvider::getData(imu_ros2::msg::VelAngTempData& message)
{
    bool success;
    m_iioWrapper.update_buffer(success);

    if(m_iioWrapper.isBufferLoaded() == false)
        return false;

    if(success)
    {
        message.delta_vel.x = m_iioWrapper.getVelocityX();
        message.delta_vel.y = m_iioWrapper.getVelocityY();
        message.delta_vel.z = m_iioWrapper.getVelocityZ();

        message.delta_angle.x = m_iioWrapper.getRotX();
        message.delta_angle.y = m_iioWrapper.getRotY();
        message.delta_angle.z = m_iioWrapper.getRotZ();

        message.temp = m_iioWrapper.getTemperature();

        message.count = m_iioWrapper.count();
    }

    return success;
}
