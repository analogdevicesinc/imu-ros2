/*******************************************************************************
 *   @file   ros_publisher_group_interface.h
 *   @brief  Interface for a group of publishers.
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

#ifndef ROS_PUBLISHER_GROUP_INTERFACE_H
#define ROS_PUBLISHER_GROUP_INTERFACE_H

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "imu_ros2/ros_task.h"

class AccelGyroTempRosPublisherInterface;
class ImuRosPublisherInterface;
class VelAngTempRosPublisherInterface;
class ImuFullMeasuredDataRosPublisherInterface;

class RosPublisherGroupInterface : public RosTask
{
public:
  /**
   * \brief Constructor for RosPublisherGroupInterface.
   *
   * This is the default constructor for interface
   *  RosPublisherGroupInterface.
   *
   */
  RosPublisherGroupInterface() {}

  /**
   * \brief Destructor for RosPublisherGroupInterface.
   *
   * This is a virtual destructor for RosPublisherGroupInterface.
   *
   */
  virtual ~RosPublisherGroupInterface() {}

  /**
   * @brief Initialize class with ros2 Node instance.
   *
   * This function initialize the class that inherit
   * this interface wiht a ros2 Node instance.
   *
   * @param node The ros2 Node instance
   */
  virtual void init(std::shared_ptr<rclcpp::Node> & node) = 0;

  virtual void setAccelGyroTempRosPublisher(AccelGyroTempRosPublisherInterface * accelGyroTempRosPublisher) = 0;

  virtual void setVelAngTempRosPublisher(VelAngTempRosPublisherInterface * velAngTempRosPublisher) = 0;

  virtual void setImuRosPublisher(ImuRosPublisherInterface * imuRosPublisher) = 0;

  virtual void setImuFullMeasuredDataRosPublisher(ImuFullMeasuredDataRosPublisherInterface * imuFullMeasuredDataRosPublisher) = 0;


protected:
  /*! The ros2 Node data member */
  std::shared_ptr<rclcpp::Node> m_node;
};

#endif  // ROS_PUBLISHER_GROUP_INTERFACE_H
