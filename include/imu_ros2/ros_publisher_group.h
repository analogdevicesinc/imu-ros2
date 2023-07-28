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

class RosPublisherGroup : public RosPublisherGroupInterface
{
public:
  /**
   * \brief Constructor for RosPublisherGroup.
   *
   * This is the default constructor for class
   *  RosPublisherGroup.
   *
   * @param node The ros2 Node instance.
   */
  RosPublisherGroup(std::shared_ptr<rclcpp::Node> & node);

  /**
   * \brief Destructor for RosPublisherGroup.
   *
   * This is a destructor for RosPublisherGroup.
   *
   */
  ~RosPublisherGroup();

  /**
   * @brief Initialize class with ros2 Node instance.
   *
   * This function initialize the class that inherit
   * this interface wiht a ros2 Node instance.
   *
   * @param node The ros2 Node instance.
   */
  void init(std::shared_ptr<rclcpp::Node> & node) override;

  void setAccelGyroTempRosPublisher(AccelGyroTempRosPublisherInterface * accelGyroTempRosPublisher) override;

  void setVelAngTempRosPublisher(VelAngTempRosPublisherInterface * velAngTempRosPublisher) override;

  void setImuRosPublisher(ImuRosPublisherInterface * imuRosPublisher) override;

  void setImuFullMeasuredDataRosPublisher(ImuFullMeasuredDataRosPublisherInterface * imuFullMeasuredDataRosPublisher)  override;

  /**
   * @brief Read from message provider and write on topic
   *
   * Run on thread the reading from message provider and write
   * on publisher the data.
   *
   */
  void run() override;

private:
  /*! This variable retain a message provider */
  AccelGyroTempRosPublisherInterface * m_accelGyroTempRosPublisher;
  VelAngTempRosPublisherInterface * m_velAngTempRosPublisher;
  ImuRosPublisherInterface * m_imuRosPublisher;
  ImuFullMeasuredDataRosPublisherInterface * m_imuFullMeasuredDataRosPublisher;

};

#endif  // ROS_PUBLISHER_GROUP_H
