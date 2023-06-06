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
    if(m_iioWrapper.isBufferLoaded() == false)
        return false;

    message.lost_samples_count = m_iioWrapper.lost_samples_count();
    message.diag_checksum_error_flag = m_iioWrapper.diag_checksum_error_flag();
    message.diag_flash_memory_write_count_exceeded_error = m_iioWrapper.diag_flash_memory_write_count_exceeded_error();
    message.diag_acceleration_self_test_error = m_iioWrapper.diag_acceleration_self_test_error();
    message.diag_gyroscope2_self_test_error = m_iioWrapper.diag_gyroscope2_self_test_error();
    message.diag_gyroscope1_self_test_error = m_iioWrapper.diag_gyroscope1_self_test_error();
    message.diag_clock_error = m_iioWrapper.diag_clock_error();
    message.diag_flash_memory_test_error = m_iioWrapper.diag_flash_memory_test_error();
    message.diag_sensor_self_test_error = m_iioWrapper.diag_sensor_self_test_error();
    message.diag_standby_mode = m_iioWrapper.diag_standby_mode();
    message.diag_spi_communication_error = m_iioWrapper.diag_spi_communication_error();
    message.diag_flash_memory_update_error = m_iioWrapper.diag_flash_memory_update_error();
    message.diag_data_path_overrun = m_iioWrapper.diag_data_path_overrun();
    message.flash_counter = m_iioWrapper.flash_counter();

    return true;
}
