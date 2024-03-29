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

#include "adis_data_access.h"
#include "imu_ros2/ros_task.h"

class AccelGyroTempRosPublisherInterface;
class ImuRosPublisherInterface;
class VelAngTempRosPublisherInterface;
class ImuFullMeasuredDataRosPublisherInterface;
class ImuControlParameters;

/**
 * @brief Interface for publisher group.
 */
class RosPublisherGroupInterface : public RosTask
{
public:
  /**
   * @brief Constructor for RosPublisherGroupInterface.
   */
  RosPublisherGroupInterface() {}

  /**
   * @brief Destructor for RosPublisherGroupInterface.
   */
  virtual ~RosPublisherGroupInterface() {}

  /**
   * @brief Sets the accelGyroTempRosPublisher publisher in the publisher group.
   * @param accelGyroTempRosPublisher The publisher to be set in the group.
   */
  virtual void setAccelGyroTempRosPublisher(
    AccelGyroTempRosPublisherInterface * accelGyroTempRosPublisher) = 0;

#ifdef ADIS_HAS_DELTA_BURST
  /**
   * @brief Sets the velAngTempRosPublisher publisher in the publisher group.
   * @param velAngTempRosPublisher The publisher to be set in the group.
   */
  virtual void setVelAngTempRosPublisher(
    VelAngTempRosPublisherInterface * velAngTempRosPublisher) = 0;
#endif

  /**
   * @brief Sets the imuRosPublisher publisher in the publisher group.
   * @param imuRosPublisher The publisher to be set in the group.
   */
  virtual void setImuRosPublisher(ImuRosPublisherInterface * imuRosPublisher) = 0;

  /**
   * @brief Sets the imuFullMeasuredDataRosPublisher publisher in the publisher
   * group.
   * @param imuFullMeasuredDataRosPublisher The imuFullMeasuredDataRosPublisher
   * to be set in the group.
   */
  virtual void setImuFullMeasuredDataRosPublisher(
    ImuFullMeasuredDataRosPublisherInterface * imuFullMeasuredDataRosPublisher) = 0;

  /**
   * @brief Sets the imuControlParameters instance in the publisher group.
   * @param imuControlParameters The imuControlParameters instance to be set in
   * the group.
   */
  virtual void setImuControlParameters(ImuControlParameters * imuControlParameters) = 0;

protected:
  /*! The ros2 Node data member. */
  std::shared_ptr<rclcpp::Node> m_node;
};

#endif  // ROS_PUBLISHER_GROUP_INTERFACE_H
