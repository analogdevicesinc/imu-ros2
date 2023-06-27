/*******************************************************************************
 *   @file   imu_ros_publisher_interface.h
 *   @brief  Interface for IMU ros standard message publisher.
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

#ifndef ACCELERATION_ROS_PUBLISHER_INTERFACE_H
#define ACCELERATION_ROS_PUBLISHER_INTERFACE_H

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "imu_ros2/ros_task.h"

class ImuDataProviderInterface;

class ImuRosPublisherInterface : public RosTask
{
public:
  ImuRosPublisherInterface() {}
  virtual ~ImuRosPublisherInterface() {}

  virtual void init(std::shared_ptr<rclcpp::Node> & node) = 0;
  virtual void setMessageProvider(ImuDataProviderInterface * dataProvider) = 0;

protected:
  std::shared_ptr<rclcpp::Node> m_node;
};

#endif  // ACCELERATION_ROS_PUBLISHER_INTERFACE_H
