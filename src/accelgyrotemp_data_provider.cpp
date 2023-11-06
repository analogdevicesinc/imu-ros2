/*******************************************************************************
 *   @file   accelgyrotemp_data_provider.cpp
 *   @brief  Implementation for providing acceleration, gyroscope and
 *           temperature data provider.
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

#include "imu_ros2/accelgyrotemp_data_provider.h"

AccelGyroTempDataProvider::AccelGyroTempDataProvider() {}

AccelGyroTempDataProvider::~AccelGyroTempDataProvider() {}

bool AccelGyroTempDataProvider::getData(imu_ros2::msg::AccelGyroTempData & message)
{
  if (!m_iio_wrapper.updateBuffer(ACCEL_GYRO_BUFFERED_DATA)) return false;

  message.linear_acceleration.x = m_iio_wrapper.getBuffLinearAccelerationX();
  message.linear_acceleration.y = m_iio_wrapper.getBuffLinearAccelerationY();
  message.linear_acceleration.z = m_iio_wrapper.getBuffLinearAccelerationZ();

  message.angular_velocity.x = m_iio_wrapper.getBuffAngularVelocityX();
  message.angular_velocity.y = m_iio_wrapper.getBuffAngularVelocityY();
  message.angular_velocity.z = m_iio_wrapper.getBuffAngularVelocityZ();

  message.temperature = m_iio_wrapper.getBuffTemperature();

  message.header.frame_id = "accelgyrotempdata";
  m_iio_wrapper.getBuffSampleTimestamp(message.header.stamp.sec, message.header.stamp.nanosec);

  return true;
}
