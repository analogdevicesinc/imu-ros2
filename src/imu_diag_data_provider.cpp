/*******************************************************************************
 *   @file   imu_diag_data_provider.cpp
 *   @brief  Implementation for providing diagnosis data for adis.
 *   @author Vasile Holonec (Vasile.Holonec@analog.com)
 *******************************************************************************
 * Copyright 2023(c) Analog Devices, Inc.
 *
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
 ******************************************************************************/

#include "imu_ros2/imu_diag_data_provider.h"

#include <rclcpp/rclcpp.hpp>

ImuDiagDataProvider::ImuDiagDataProvider() {}

ImuDiagDataProvider::~ImuDiagDataProvider() {}


bool ImuDiagDataProvider::getData(imu_ros2::msg::ImuDiagDataadis1646x & message)
{
  message.header.frame_id = "imudiagdataadis1646x";

  if (!m_iio_wrapper.diag_data_path_overrun(message.diag_data_path_overrun)) return false;

  if (!m_iio_wrapper.diag_flash_memory_update_error(message.diag_flash_memory_update_error))
    return false;

  if (!m_iio_wrapper.diag_spi_communication_error(message.diag_spi_communication_error))
    return false;

  if (!m_iio_wrapper.diag_standby_mode(message.diag_standby_mode)) return false;

  if (!m_iio_wrapper.diag_sensor_self_test_error(message.diag_sensor_self_test_error)) return false;

  if (!m_iio_wrapper.diag_flash_memory_test_error(message.diag_flash_memory_test_error))
    return false;

  if (!m_iio_wrapper.diag_clock_error(message.diag_clock_error)) return false;

  if (!m_iio_wrapper.diag_flash_memory_write_count_exceeded_error(
        message.diag_flash_memory_write_count_exceeded_error))
    return false;

  return m_iio_wrapper.flash_counter(message.flash_counter);
}


bool ImuDiagDataProvider::getData(imu_ros2::msg::ImuDiagDataadis1647x & message)
{
  message.header.frame_id = "imudiagdataadis1647x";

  if (!m_iio_wrapper.diag_data_path_overrun(message.diag_data_path_overrun)) return false;

  if (!m_iio_wrapper.diag_flash_memory_update_error(message.diag_flash_memory_update_error))
    return false;

  if (!m_iio_wrapper.diag_spi_communication_error(message.diag_spi_communication_error))
    return false;

  if (!m_iio_wrapper.diag_standby_mode(message.diag_standby_mode)) return false;

  if (!m_iio_wrapper.diag_sensor_self_test_error(message.diag_sensor_self_test_error)) return false;

  if (!m_iio_wrapper.diag_flash_memory_test_error(message.diag_flash_memory_test_error))
    return false;

  if (!m_iio_wrapper.diag_clock_error(message.diag_clock_error)) return false;

  if (!m_iio_wrapper.diag_flash_memory_write_count_exceeded_error(
        message.diag_flash_memory_write_count_exceeded_error))
    return false;

  return m_iio_wrapper.flash_counter(message.flash_counter);
}


bool ImuDiagDataProvider::getData(imu_ros2::msg::ImuDiagDataadis1650x & message)
{
  message.header.frame_id = "imudiagdataadis1650x";

  if (!m_iio_wrapper.diag_data_path_overrun(message.diag_data_path_overrun)) return false;

  if (!m_iio_wrapper.diag_flash_memory_update_error(message.diag_flash_memory_update_error))
    return false;

  if (!m_iio_wrapper.diag_spi_communication_error(message.diag_spi_communication_error))
    return false;

  if (!m_iio_wrapper.diag_standby_mode(message.diag_standby_mode)) return false;

  if (!m_iio_wrapper.diag_sensor_self_test_error(message.diag_sensor_self_test_error)) return false;

  if (!m_iio_wrapper.diag_flash_memory_test_error(message.diag_flash_memory_test_error))
    return false;

  if (!m_iio_wrapper.diag_clock_error(message.diag_clock_error)) return false;

  if (!m_iio_wrapper.diag_gyroscope1_self_test_error(message.diag_gyroscope1_self_test_error))
    return false;

  if (!m_iio_wrapper.diag_gyroscope2_self_test_error(message.diag_gyroscope2_self_test_error))
    return false;

  if (!m_iio_wrapper.diag_acceleration_self_test_error(message.diag_acceleration_self_test_error))
    return false;

  if (!m_iio_wrapper.diag_flash_memory_write_count_exceeded_error(
        message.diag_flash_memory_write_count_exceeded_error))
    return false;

  return m_iio_wrapper.flash_counter(message.flash_counter);
}


bool ImuDiagDataProvider::getData(imu_ros2::msg::ImuDiagDataadis1654x & message)
{
  message.header.frame_id = "imudiagdataadis1654x";

  if (!m_iio_wrapper.diag_data_path_overrun(message.diag_data_path_overrun)) return false;

  if (!m_iio_wrapper.diag_automatic_reset(message.diag_automatic_reset)) return false;

  if (!m_iio_wrapper.diag_flash_memory_update_error(message.diag_flash_memory_update_error))
    return false;

  if (!m_iio_wrapper.diag_spi_communication_error(message.diag_spi_communication_error))
    return false;

  if (!m_iio_wrapper.diag_crc_error(message.diag_crc_error)) return false;

  if (!m_iio_wrapper.diag_sensor_self_test_error(message.diag_sensor_self_test_error)) return false;

  if (!m_iio_wrapper.diag_flash_memory_test_error(message.diag_flash_memory_test_error))
    return false;

  if (!m_iio_wrapper.diag_clock_error(message.diag_clock_error)) return false;

  if (!m_iio_wrapper.diag_x_axis_gyroscope_failure(message.diag_x_axis_gyroscope_failure))
    return false;

  if (!m_iio_wrapper.diag_y_axis_gyroscope_failure(message.diag_y_axis_gyroscope_failure))
    return false;

  if (!m_iio_wrapper.diag_z_axis_gyroscope_failure(message.diag_z_axis_gyroscope_failure))
    return false;

  if (!m_iio_wrapper.diag_x_axis_accelerometer_failure(message.diag_x_axis_accelerometer_failure))
    return false;

  if (!m_iio_wrapper.diag_y_axis_accelerometer_failure(message.diag_y_axis_accelerometer_failure))
    return false;

  if (!m_iio_wrapper.diag_z_axis_accelerometer_failure(message.diag_z_axis_accelerometer_failure))
    return false;

  if (!m_iio_wrapper.diag_flash_memory_write_count_exceeded_error(
        message.diag_flash_memory_write_count_exceeded_error))
    return false;

  return m_iio_wrapper.flash_counter(message.flash_counter);
}


bool ImuDiagDataProvider::getData(imu_ros2::msg::ImuDiagDataadis1655x & message)
{
  message.header.frame_id = "imudiagdataadis1655x";

  if (!m_iio_wrapper.diag_data_path_overrun(message.diag_data_path_overrun)) return false;

  if (!m_iio_wrapper.diag_automatic_reset(message.diag_automatic_reset)) return false;

  if (!m_iio_wrapper.diag_flash_memory_update_error(message.diag_flash_memory_update_error))
    return false;

  if (!m_iio_wrapper.diag_spi_communication_error(message.diag_spi_communication_error))
    return false;

  if (!m_iio_wrapper.diag_crc_error(message.diag_crc_error)) return false;

  if (!m_iio_wrapper.diag_sensor_self_test_error(message.diag_sensor_self_test_error)) return false;

  if (!m_iio_wrapper.diag_flash_memory_test_error(message.diag_flash_memory_test_error))
    return false;

  if (!m_iio_wrapper.diag_clock_error(message.diag_clock_error)) return false;

  if (!m_iio_wrapper.diag_x_axis_gyroscope_failure(message.diag_x_axis_gyroscope_failure))
    return false;

  if (!m_iio_wrapper.diag_y_axis_gyroscope_failure(message.diag_y_axis_gyroscope_failure))
    return false;

  if (!m_iio_wrapper.diag_z_axis_gyroscope_failure(message.diag_z_axis_gyroscope_failure))
    return false;

  if (!m_iio_wrapper.diag_x_axis_accelerometer_failure(message.diag_x_axis_accelerometer_failure))
    return false;

  if (!m_iio_wrapper.diag_y_axis_accelerometer_failure(message.diag_y_axis_accelerometer_failure))
    return false;

  if (!m_iio_wrapper.diag_z_axis_accelerometer_failure(message.diag_z_axis_accelerometer_failure))
    return false;

  if (!m_iio_wrapper.diag_flash_memory_write_count_exceeded_error(
        message.diag_flash_memory_write_count_exceeded_error))
    return false;

  return m_iio_wrapper.flash_counter(message.flash_counter);
}


bool ImuDiagDataProvider::getData(imu_ros2::msg::ImuDiagDataadis1657x & message)
{
  message.header.frame_id = "imudiagdataadis1657x";

  if (!m_iio_wrapper.diag_sensor_initialization_failure(message.diag_sensor_initialization_failure))
    return false;

  if (!m_iio_wrapper.diag_data_path_overrun(message.diag_data_path_overrun)) return false;

  if (!m_iio_wrapper.diag_flash_memory_update_error(message.diag_flash_memory_update_error))
    return false;

  if (!m_iio_wrapper.diag_spi_communication_error(message.diag_spi_communication_error))
    return false;

  if (!m_iio_wrapper.diag_standby_mode(message.diag_standby_mode)) return false;

  if (!m_iio_wrapper.diag_sensor_self_test_error(message.diag_sensor_self_test_error)) return false;

  if (!m_iio_wrapper.diag_flash_memory_test_error(message.diag_flash_memory_test_error))
    return false;

  if (!m_iio_wrapper.diag_clock_error(message.diag_clock_error)) return false;

  if (!m_iio_wrapper.diag_x_axis_gyroscope_failure(message.diag_x_axis_gyroscope_failure))
    return false;

  if (!m_iio_wrapper.diag_y_axis_gyroscope_failure(message.diag_y_axis_gyroscope_failure))
    return false;

  if (!m_iio_wrapper.diag_z_axis_gyroscope_failure(message.diag_z_axis_gyroscope_failure))
    return false;

  if (!m_iio_wrapper.diag_x_axis_accelerometer_failure(message.diag_x_axis_accelerometer_failure))
    return false;

  if (!m_iio_wrapper.diag_y_axis_accelerometer_failure(message.diag_y_axis_accelerometer_failure))
    return false;

  if (!m_iio_wrapper.diag_z_axis_accelerometer_failure(message.diag_z_axis_accelerometer_failure))
    return false;

  if (!m_iio_wrapper.diag_aduc_mcu_fault(message.diag_aduc_mcu_fault)) return false;

  if (!m_iio_wrapper.diag_flash_memory_write_count_exceeded_error(
        message.diag_flash_memory_write_count_exceeded_error))
    return false;

  return m_iio_wrapper.flash_counter(message.flash_counter);
}
