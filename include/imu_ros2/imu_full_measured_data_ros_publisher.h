/***************************************************************************//**
 *   @file   imu_full_measured_data_ros_publisher.h
 *   @brief  Header for acceleration, gyroscope, temperature, delta velocity,
 *           delta angle and temperature publisher.
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

#ifndef IMU_FULL_MEASURED_DATA_ROS_PUBLISHER_H
#define IMU_FULL_MEASURED_DATA_ROS_PUBLISHER_H

#include "imu_ros2/imu_full_measured_data_ros_publisher_interface.h"
#include "imu_ros2/imu_full_measured_data_provider_interface.h"

#include <rclcpp/rclcpp.hpp>

class ImuFullMeasuredDataRosPublisher : public ImuFullMeasuredDataRosPublisherInterface
{

public:
  ImuFullMeasuredDataRosPublisher(std::shared_ptr<rclcpp::Node>& node);
  ~ImuFullMeasuredDataRosPublisher();

  void init(std::shared_ptr<rclcpp::Node>& node) override;
  void setMessageProvider(ImuFullMeasuredDataProviderInterface* dataProvider) override;

  void run() override;

private:

  ImuFullMeasuredDataProviderInterface* m_data_provider;
  rclcpp::Publisher<imu_ros2::msg::ImuFullMeasuredData>::SharedPtr m_publisher;
  imu_ros2::msg::ImuFullMeasuredData m_message;
};

#endif // IMU_FULL_MEASURED_DATA_ROS_PUBLISHER_H
