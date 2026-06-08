/*******************************************************************************
 *   @file   imu_ros_publisher.cpp
 *   @brief  Implementation for providing IMU ros standard data.
 *   @author Vasile Holonec (Vasile.Holonec@analog.com)
*******************************************************************************/
// Copyright 2023 Analog Devices, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "adi_imu/imu_data_provider.h"

#include <sensor_msgs/msg/imu.hpp>

namespace adi_imu
{

ImuDataProvider::ImuDataProvider() : m_frame_id("imu") {}

ImuDataProvider::~ImuDataProvider() {}

void ImuDataProvider::setFrameId(const std::string & frame_id) { m_frame_id = frame_id; }

bool ImuDataProvider::getData(sensor_msgs::msg::Imu & message)
{
  if (!m_iio_wrapper.updateBuffer(ACCEL_GYRO_BUFFERED_DATA)) return false;

  message.linear_acceleration.x = m_iio_wrapper.getBuffLinearAccelerationX();
  message.linear_acceleration.y = m_iio_wrapper.getBuffLinearAccelerationY();
  message.linear_acceleration.z = m_iio_wrapper.getBuffLinearAccelerationZ();

  message.angular_velocity.x = m_iio_wrapper.getBuffAngularVelocityX();
  message.angular_velocity.y = m_iio_wrapper.getBuffAngularVelocityY();
  message.angular_velocity.z = m_iio_wrapper.getBuffAngularVelocityZ();

  message.header.frame_id = m_frame_id;
  m_iio_wrapper.getBuffSampleTimestamp(message.header.stamp.sec, message.header.stamp.nanosec);

  // No orientation provided direclty by the IMU
  message.orientation_covariance[0] = -1;

  // Handle covariance if provider is set
  if (m_covariance_provider) {
    //Feed sample for calibration/adaptation
    adi_imu::Vec3 accel = message.linear_acceleration;
    adi_imu::Vec3 gyro = message.angular_velocity;

    m_covariance_provider->addSample(accel, gyro);
    if (m_covariance_provider->isReady()) {
      // Copy covariance to message
      auto accel_cov = m_covariance_provider->getAccelCovariance();
      auto gyro_cov = m_covariance_provider->getGyroCovariance();

      std::copy(accel_cov.begin(), accel_cov.end(), message.linear_acceleration_covariance.begin());
      std::copy(gyro_cov.begin(), gyro_cov.end(), message.angular_velocity_covariance.begin());
    } else {
      // Set to zero to indicate unknown covariance during calibration
      std::fill(
        message.linear_acceleration_covariance.begin(),
        message.linear_acceleration_covariance.end(), 0);
      std::fill(
        message.angular_velocity_covariance.begin(), message.angular_velocity_covariance.end(), 0);
    }
  } else {
    // No covariance provider - set to unknown
    std::fill(
      message.linear_acceleration_covariance.begin(), message.linear_acceleration_covariance.end(),
      0);
    std::fill(
      message.angular_velocity_covariance.begin(), message.angular_velocity_covariance.end(), 0);
  }

  return true;
}

// Method for setting the covariance provider
void ImuDataProvider::setCovarianceProvider(ImuCovarianceInterface * provider)
{
  m_covariance_provider = provider;
}

}  // namespace adi_imu
