/*******************************************************************************
 *   @file   imu_16505_diag_ros_publisher_interface.h
 *   @brief  Interface for adis16505 diagnosis publisher.
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

#ifndef IMU_16505_DIAG_ROS_PUBLISHER_INTERFACE_H
#define IMU_16505_DIAG_ROS_PUBLISHER_INTERFACE_H

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "imu_ros2/ros_task.h"

class Imu16505DiagDataProviderInterface;

/**
 * \brief Interface for adis16505 diag ros publisher.
 *
 * This interface initializes the ros Node class.
 * Also it set message provider with a variable that is
 * a type of Imu16505DiagDataProviderInterface.
 */
class Imu16505DiagRosPublisherInterface : public RosTask
{
public:
  /**
   * \brief Constructor for Imu16505DiagRosPublisherInterface.
   *
   * This is the default constructor for interface
   *  Imu16505DiagRosPublisherInterface.
   *
   */
  Imu16505DiagRosPublisherInterface() {}

  /**
   * \brief Destructor for Imu16505DiagRosPublisherInterface.
   *
   * This is a virtual destructor for Imu16505DiagRosPublisherInterface.
   *
   */
  virtual ~Imu16505DiagRosPublisherInterface() {}

  /**
   * @brief Initialize class with ros2 Node instance.
   *
   * This function initialize the class that inherit
   * this interface wiht a ros2 Node instance.
   *
   * @param node The ros2 Node instance
   */
  virtual void init(std::shared_ptr<rclcpp::Node> & node) = 0;

  /**
   * @brief Set message provider.
   *
   * This function set data message provider with a variable that
   * inherit Imu16505DiagDataProviderInterface.
   *
   * @param dataProvider Data message provider.
   */
  virtual void setMessageProvider(Imu16505DiagDataProviderInterface * dataProvider) = 0;

protected:
  std::shared_ptr<rclcpp::Node> m_node; /**< The ros2 Node data member */
};

#endif  // IMU_16505_DIAG_ROS_PUBLISHER_INTERFACE_H
