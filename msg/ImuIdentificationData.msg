################################################################################
#   @file   ImuIdentificationData.msg
#   @brief  Definition of ImuIdentificationData message
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

# firmware revision for the imu internal firmware
string firmware_revision

# firmware date for the imu internal firmware
string firmware_date

# imu product identification
uint32 product_id

# imu lot specific serial number
uint32 serial_number

# imu gyroscope measurement range
string gyroscope_measurement_range
