/*******************************************************************************
 *   @file   imu_diag_ros_publisher_interface.h
 *   @brief  Interface for adis diagnosis publisher.
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

#ifndef IMU_DIAG_ROS_PUBLISHER_INTERFACE_H
#define IMU_DIAG_ROS_PUBLISHER_INTERFACE_H

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "adi_imu/ros_task.h"

class ImuDiagDataProviderInterface;

/**
 * @brief Interface for diagnosis publisher for adis chips.
 */
class ImuDiagRosPublisherInterface : public RosTask
{
public:
  /**
   * @brief Constructor for ImuDiagRosPublisherInterface.
   */
  ImuDiagRosPublisherInterface() {}

  /**
   * @brief Destructor for ImuDiagRosPublisherInterface.
   */
  virtual ~ImuDiagRosPublisherInterface() {}

  /**
   * @brief Set the message data provider.
   * @param dataProvider Data provider.
   */
  virtual void setMessageProvider(ImuDiagDataProviderInterface * dataProvider) = 0;

protected:
  /*! The ros2 Node data member. */
  std::shared_ptr<rclcpp::Node> m_node;
};

#endif  // IMU_DIAG_ROS_PUBLISHER_INTERFACE_H
