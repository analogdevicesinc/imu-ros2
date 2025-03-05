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

#include "adi_imu/velangtemp_data_provider.h"

VelAngTempDataProvider::VelAngTempDataProvider() {}

VelAngTempDataProvider::~VelAngTempDataProvider() {}

bool VelAngTempDataProvider::getData(adi_imu::msg::VelAngTempData & message)
{
#ifdef ADIS_HAS_DELTA_BURST
  if (!m_iio_wrapper.updateBuffer(DELTAVEL_DELTAANG_BUFFERED_DATA)) return false;

  message.delta_angle.x = m_iio_wrapper.getBuffDeltaAngleX();
  message.delta_angle.y = m_iio_wrapper.getBuffDeltaAngleY();
  message.delta_angle.z = m_iio_wrapper.getBuffDeltaAngleZ();

  message.delta_velocity.x = m_iio_wrapper.getBuffDeltaVelocityX();
  message.delta_velocity.y = m_iio_wrapper.getBuffDeltaVelocityY();
  message.delta_velocity.z = m_iio_wrapper.getBuffDeltaVelocityZ();

  message.temperature = m_iio_wrapper.getBuffTemperature();

  message.header.frame_id = "velangtempdata";
  m_iio_wrapper.getBuffSampleTimestamp(message.header.stamp.sec, message.header.stamp.nanosec);

  return true;
#else
  return false;
#endif
}
