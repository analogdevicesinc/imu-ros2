/***************************************************************************//**
 *   @file   imu_16505_diag_ros_publisher.h
 *   @brief  Header for adis16505 diagnosis publisher.
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

#ifndef IMU_16505_DIAG_ROS_SUBSCRIBER_H
#define IMU_16505_DIAG_ROS_SUBSCRIBER_H

#include "imu_ros2/imu_16505_diag_ros_publisher_interface.h"
#include "imu_ros2/imu_16505_diag_data_provider_interface.h"

#include <rclcpp/rclcpp.hpp>

class Imu16505DiagRosPublisher : public Imu16505DiagRosPublisherInterface
{

public:
  Imu16505DiagRosPublisher(std::shared_ptr<rclcpp::Node>& node);
  ~Imu16505DiagRosPublisher();

  void init(std::shared_ptr<rclcpp::Node>& node) override;
  void setMessageProvider(Imu16505DiagDataProviderInterface* dataProvider) override;

  void run() override;

private:
  Imu16505DiagDataProviderInterface* m_data_provider;
  rclcpp::Publisher<imu_ros2::msg::Imu16505DiagData>::SharedPtr m_publisher;
  imu_ros2::msg::Imu16505DiagData m_message;
};

#endif // IMU_16505_DIAG_ROS_SUBSCRIBER_H
