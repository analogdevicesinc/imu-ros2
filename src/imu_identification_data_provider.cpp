/*******************************************************************************
 *   @file   imu_identification_data_provider.cpp
 *   @brief  Implementation for imu identification data provider.
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

#include "imu_ros2/imu_identification_data_provider.h"

ImuIdentificationDataProvider::ImuIdentificationDataProvider() {}

ImuIdentificationDataProvider::~ImuIdentificationDataProvider() {}

bool ImuIdentificationDataProvider::getData(imu_ros2::msg::ImuIdentificationData & message)
{
  if (!m_iio_wrapper.firmware_revision(message.firmware_revision)) return false;

  if (!m_iio_wrapper.firmware_date(message.firmware_date)) return false;

  if (!m_iio_wrapper.product_id(message.product_id)) return false;

  if (!m_iio_wrapper.serial_number(message.serial_number)) return false;

  return m_iio_wrapper.gyroscope_measurement_range(message.gyroscope_measurement_range);
}
