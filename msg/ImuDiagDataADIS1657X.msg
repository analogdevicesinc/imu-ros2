################################################################################
#   @file   ImuDiagData.msg
#   @brief  Definition of ImuDiagData message
#   @author RBolboac (ramona.bolboaca@analog.com)
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

# if true, the imu failed to initialize properly
bool diag_sensor_initialization_failure

# if true, one of the datapaths experienced an overrun condition
bool diag_data_path_overrun

# if true, the most recent imu memory flash failed
bool diag_flash_memory_update_error

# if true, the total number of SPI SCLK cycles is not equal to an integer multiple of 16
bool diag_spi_communication_error

# if true, the imu voltage across VDD and GND < 2.9V, which causes data processing to stop
bool diag_standby_mode

# if true, the imu self-test routine failed
bool diag_sensor_self_test_error

# if true, there is a failure in imu flash memory
bool diag_flash_memory_test_error

# if true, the internal data sampling clock does not synchronize with the external clock (only in scaled sync mode)
bool diag_clock_error

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

# if true, a fault occurred in the imu microcontroller
bool diag_aduc_mcu_fault

# if true, the imu flash memory was written more times than the data-sheet specified endurance
bool diag_flash_memory_write_count_exceeded_error

# the value of the imu flash writes
uint32 flash_counter
