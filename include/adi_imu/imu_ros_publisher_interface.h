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

class ImuDataProviderInterface;

/**
 * @brief Interface for standard message sensor_msgs::msg::Imu publisher.
 */
class ImuRosPublisherInterface
{
public:
  /**
   * @brief Constructor for ImuRosPublisherInterface.
   */
  ImuRosPublisherInterface() {}

  /**
   * @brief Destructor for ImuRosPublisherInterface.
   */
  virtual ~ImuRosPublisherInterface() {}

  /**
   * @brief Set the message data provider.
   * @param dataProvider Data provider.
   */
  virtual void setMessageProvider(ImuDataProviderInterface * dataProvider) = 0;

  /**
   * @brief Publish the sensor_msgs::msg::Imu message.
   */
  virtual void publish() = 0;

protected:
  /*! The ros2 Node data member. */
  std::shared_ptr<rclcpp::Node> m_node;
};

#endif  // ACCELERATION_ROS_PUBLISHER_INTERFACE_H
