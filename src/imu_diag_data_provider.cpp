/***************************************************************************//**
*   @file   imu_diag_data_provider.cpp
*   @brief  Implementation for imu diag data
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

#include "imu_ros2/imu_diag_data_provider.h"
#include <rclcpp/rclcpp.hpp>

ImuDiagDataProvider::ImuDiagDataProvider()
{
    init();
}

ImuDiagDataProvider::~ImuDiagDataProvider()
{

}

void ImuDiagDataProvider::init()
{
    // initialize a library
}

bool ImuDiagDataProvider::getData(imu_ros2::msg::ImuDiagData& message)
{
    bool success;
    success = m_iioWrapper.lost_samples_count(message.lost_samples_count);
    if(!success)
        return false;

    success = m_iioWrapper.diag_checksum_error_flag(message.diag_checksum_error_flag);
    if(!success)
        return false;

    success = m_iioWrapper.diag_flash_memory_write_count_exceeded_error(
                message.diag_flash_memory_write_count_exceeded_error);
    if(!success)
        return false;

    success = m_iioWrapper.diag_acceleration_self_test_error(message.diag_acceleration_self_test_error);
    if(!success)
        return false;

    success = m_iioWrapper.diag_gyroscope2_self_test_error(message.diag_gyroscope2_self_test_error);
    if(!success)
        return false;

    success = m_iioWrapper.diag_gyroscope1_self_test_error(message.diag_gyroscope1_self_test_error);
    if(!success)
        return false;

    success = m_iioWrapper.diag_clock_error(message.diag_clock_error);
    if(!success)
        return false;

    success = m_iioWrapper.diag_flash_memory_test_error(message.diag_flash_memory_test_error);
    if(!success)
        return false;

    success = m_iioWrapper.diag_sensor_self_test_error(message.diag_sensor_self_test_error);
    if(!success)
        return false;

    success = m_iioWrapper.diag_standby_mode(message.diag_standby_mode);
    if(!success)
        return false;

    success = m_iioWrapper.diag_spi_communication_error(message.diag_spi_communication_error);
    if(!success)
        return false;

    success = m_iioWrapper.diag_flash_memory_update_error(message.diag_flash_memory_update_error);
    if(!success)
        return false;

    success = m_iioWrapper.diag_data_path_overrun(message.diag_data_path_overrun);
    if(!success)
        return false;

    return m_iioWrapper.flash_counter(message.flash_counter);
}
