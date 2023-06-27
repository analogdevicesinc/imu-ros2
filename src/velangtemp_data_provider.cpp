/*******************************************************************************
 *   @file   velangtemp_data_provider.cpp
 *   @brief  Implementation for temperature, delta velocity and delta angle
 *           data provider.
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

#include "imu_ros2/velangtemp_data_provider.h"

VelAngTempDataProvider::VelAngTempDataProvider() {}

VelAngTempDataProvider::~VelAngTempDataProvider() {}

bool VelAngTempDataProvider::enableBufferedDataOutput()
{
  return (m_iio_wrapper.update_burst_data_selection(1) == true);
}

bool VelAngTempDataProvider::getData(imu_ros2::msg::VelAngTempData & message)
{
  if (!m_iio_wrapper.updateBuffer()) return false;

  message.delta_angle.x = m_iio_wrapper.getBuffDeltaAngleX();
  message.delta_angle.y = m_iio_wrapper.getBuffDeltaAngleY();
  message.delta_angle.z = m_iio_wrapper.getBuffDeltaAngleZ();

  message.delta_velocity.x = m_iio_wrapper.getBuffDeltaVelocityX();
  message.delta_velocity.y = m_iio_wrapper.getBuffDeltaVelocityY();
  message.delta_velocity.z = m_iio_wrapper.getBuffDeltaVelocityZ();

  message.temperature = m_iio_wrapper.getBuffTemperature();

  message.sample_count = m_iio_wrapper.getBuffSampleCount();

  return true;
}
