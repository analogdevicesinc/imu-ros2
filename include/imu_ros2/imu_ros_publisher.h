/*******************************************************************************
 *   @file   imu_ros_publisher.h
 *   @brief  Header for IMU ros standard message publisher.
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

#ifndef IMU_ROS_PUBLISHER_H
#define IMU_ROS_PUBLISHER_H

#include <rclcpp/rclcpp.hpp>

#include "imu_ros2/imu_data_provider_interface.h"
#include "imu_ros2/imu_ros_publisher_interface.h"

class ImuRosPublisher : public ImuRosPublisherInterface
{
public:
  ImuRosPublisher(std::shared_ptr<rclcpp::Node> & node);
  ~ImuRosPublisher();

  void init(std::shared_ptr<rclcpp::Node> & node) override;
  void setMessageProvider(ImuDataProviderInterface * dataProvider) override;

  void run() override;

private:
  ImuDataProviderInterface * m_data_provider;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_publisher;
  sensor_msgs::msg::Imu m_message;
};

#endif  // IMU_ROS_PUBLISHER_H
