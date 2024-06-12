################################################################################
#   @file   ImuDiagData.msg
#   @brief  Definition of ImuDiagData message
#   @author Robert Budai (robert.budai@analog.com)
################################################################################
# Copyright 2023(c) Analog Devices, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

# message header
std_msgs/Header header

# if true, one of the datapaths experienced an overrun condition
bool diag_data_path_overrun

# if true, the most recent imu memory flash failed
bool diag_flash_memory_update_error

# if true, sensor automatically reset themselves to clear an issue
bool diag_automatic_reset

# if true, while operating in scaled sync mode, indicates the sampling time is not scaling correctly
bool diag_clock_error

# if true, indicates the occurrence of a processing overrun
bool diag_processing_overrun

# if true, the most recent imu memory flash failed
bool diag_flash_memory_test_error

# if true, indicates the failure of the inertial sensor
bool diag_sensor_self_test_error

#if true, indicates communication error on SPI interface
bool diag_spi_communication_error

# if true, indicates failure on CRC calculation
bool diag_crc_error

#if true, indicates the device booted up using the backup memory bank
bool diag_boot_mem_fail

# if true, a failure occurred on x axis gyroscope
bool diag_x_axis_gyroscope_failure

# if true, a failure occurred on y axis gyroscope
bool diag_y_axis_gyroscope_failure

# if true, a failure occurred on z axis gyroscope
bool diag_z_axis_gyroscope_failure

# if true, a failure occurred on x axis accelerometer
bool diag_x_axis_accelerometer_failure

# if true, a failure occurred on y axis accelerometer
bool diag_y_axis_accelerometer_failure

# if true, a failure occurred on z axis accelerometer
bool diag_z_axis_accelerometer_failure

# if true, the imu flash memory was written more times than the data-sheet specified endurance
bool diag_flash_memory_write_count_exceeded_error

# the value of the imu flash writes
uint32 flash_counter
