/*******************************************************************************
 *   @file   imu_full_measured_data_provider.cpp
 *   @brief  Implementation for acceleration, gyroscope, temperature, delta
 *           velocity, delta angle and temperature data provider.
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

#include "imu_ros2/imu_full_measured_data_provider.h"

ImuFullMeasuredDataProvider::ImuFullMeasuredDataProvider() {}

ImuFullMeasuredDataProvider::~ImuFullMeasuredDataProvider() {}

bool ImuFullMeasuredDataProvider::getData(imu_ros2::msg::ImuFullMeasuredData & data)
{
  m_iio_wrapper.stopBufferAcquisition();

  data.header.frame_id = "imufullmeasureddata";

  if (!m_iio_wrapper.getConvertedLinearAccelerationX(data.linear_acceleration.x)) return false;

  if (!m_iio_wrapper.getConvertedLinearAccelerationY(data.linear_acceleration.y)) return false;

  if (!m_iio_wrapper.getConvertedLinearAccelerationZ(data.linear_acceleration.z)) return false;

  if (!m_iio_wrapper.getConvertedAngularVelocityX(data.angular_velocity.x)) return false;

  if (!m_iio_wrapper.getConvertedAngularVelocityY(data.angular_velocity.y)) return false;

  if (!m_iio_wrapper.getConvertedAngularVelocityZ(data.angular_velocity.z)) return false;

  if (!m_iio_wrapper.getConvertedDeltaVelocityX(data.delta_velocity.x)) return false;

  if (!m_iio_wrapper.getConvertedDeltaVelocityY(data.delta_velocity.y)) return false;

  if (!m_iio_wrapper.getConvertedDeltaVelocityZ(data.delta_velocity.z)) return false;

  if (!m_iio_wrapper.getConvertedDeltaAngleX(data.delta_angle.x)) return false;

  if (!m_iio_wrapper.getConvertedDeltaAngleY(data.delta_angle.y)) return false;

  if (!m_iio_wrapper.getConvertedDeltaAngleZ(data.delta_angle.z)) return false;

  return m_iio_wrapper.getConvertedTemperature(data.temperature);
}
