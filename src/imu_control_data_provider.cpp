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
    message.anglvel_x_calibbias = m_iioWrapper.anglvel_x_calibbias();
    message.anglvel_y_calibbias = m_iioWrapper.anglvel_y_calibbias();
    message.anglvel_z_calibbias = m_iioWrapper.anglvel_z_calibbias();
    message.accel_x_calibbias = m_iioWrapper.accel_x_calibbias();
    message.accel_y_calibbias = m_iioWrapper.accel_y_calibbias();
    message.accel_z_calibbias = m_iioWrapper.accel_z_calibbias();
    message.filter_size = m_iioWrapper.filter_size();
    message.burst_size_selection = m_iioWrapper.burst_size_selection();
    message.burst_data_selection = m_iioWrapper.burst_data_selection();
    message.linear_acceleration_compensation = m_iioWrapper.linear_acceleration_compensation();
    message.point_of_percussion_alignment = m_iioWrapper.point_of_percussion_alignment();
    message.internal_sensor_bandwidth = m_iioWrapper.internal_sensor_bandwidth();
    message.sync_mode_select = m_iioWrapper.sync_mode_select();
    message.sync_polarity = m_iioWrapper.sync_polarity();
    message.data_ready_polarity = m_iioWrapper.data_ready_polarity();
    message.sync_signal_scale = m_iioWrapper.sync_signal_scale();
    message.decimation_filter = m_iioWrapper.decimation_filter();



    return message;
}
