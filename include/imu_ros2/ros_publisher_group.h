/*******************************************************************************
 *   @file   ros_publisher_group.h
 *   @brief  Header for acceleration, gyroscope and temperature publisher.
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

#ifndef ROS_PUBLISHER_GROUP_H
#define ROS_PUBLISHER_GROUP_H

#include <rclcpp/rclcpp.hpp>

#include "imu_ros2/ros_publisher_group_interface.h"

/**
 * @brief Class ROS publisher group.
 */
class RosPublisherGroup : public RosPublisherGroupInterface
{
public:
  /**
   * @brief Constructor for RosPublisherGroup.
   * @param node The ros2 Node instance.
   */
  RosPublisherGroup(std::shared_ptr<rclcpp::Node> & node);

  /**
   * @brief Destructor for RosPublisherGroup.
   */
  ~RosPublisherGroup();

  /**
   * @brief Sets the accelGyroTempRosPublisher publisher in the publisher group.
   * @param accelGyroTempRosPublisher The publisher to be set in the group.
   */
  void setAccelGyroTempRosPublisher(
    AccelGyroTempRosPublisherInterface * accelGyroTempRosPublisher) override;


  /**
   * @brief Sets the velAngTempRosPublisher publisher in the publisher group.
   * @param velAngTempRosPublisher The publisher to be set in the group.
   */
  void setVelAngTempRosPublisher(VelAngTempRosPublisherInterface * velAngTempRosPublisher) override;


  /**
   * @brief Sets the imuRosPublisher publisher in the publisher group.
   * @param imuRosPublisher The publisher to be set in the group.
   */
  void setImuRosPublisher(ImuRosPublisherInterface * imuRosPublisher) override;

  /**
   * @brief Sets the imuFullMeasuredDataRosPublisher publisher in the publisher
   * group.
   * @param imuFullMeasuredDataRosPublisher The imuFullMeasuredDataRosPublisher
   * to be set in the group.
   */
  void setImuFullMeasuredDataRosPublisher(
    ImuFullMeasuredDataRosPublisherInterface * imuFullMeasuredDataRosPublisher) override;

  /**
   * @brief Sets the imuControlParameters instance in the publisher group.
   * @param imuControlParameters The imuControlParameters instance to be set in
   * the group.
   */
  void setImuControlParameters(ImuControlParameters * imuControlParameters) override;

  /**
   * @brief Run the thread responsible for publishing messages and for
   * performing parameter configuration.
   */
  void run() override;

private:
  /*! Variable to retain AccelGyroTemp message provider. */
  AccelGyroTempRosPublisherInterface * m_accelGyroTempRosPublisher;

  /*! Variable to retain VelAngTemp message provider. */
  VelAngTempRosPublisherInterface * m_velAngTempRosPublisher;

  /*! Variable to retain Imu message provider. */
  ImuRosPublisherInterface * m_imuRosPublisher;
  /*! Variable to retain ImuFullMeasuredData message provider. */
  ImuFullMeasuredDataRosPublisherInterface * m_imuFullMeasuredDataRosPublisher;
  /*! Variable to retain m_imuControlParameters instance. */
  ImuControlParameters * m_imuControlParameters;
};

#endif  // ROS_PUBLISHER_GROUP_H
