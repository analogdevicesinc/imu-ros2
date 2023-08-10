/*******************************************************************************
 *   @file   accelgyrotemp_ros_publisher.h
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

#ifndef ACCELGYROTEMP_ROS_SUBSCRIBER_H
#define ACCELGYROTEMP_ROS_SUBSCRIBER_H

#include <rclcpp/rclcpp.hpp>

#include "imu_ros2/accelgyrotemp_data_provider_interface.h"
#include "imu_ros2/accelgyrotemp_ros_publisher_interface.h"

/**
 * @brief Class for acceleration, angular velocity and temperature publisher.
 */
class AccelGyroTempRosPublisher : public AccelGyroTempRosPublisherInterface
{
public:
  /**
   * @brief Constructor for AccelGyroTempRosPublisher.
   * @param node The ros2 Node instance.
   */
  AccelGyroTempRosPublisher(std::shared_ptr<rclcpp::Node> & node);

  /**
   * @brief Destructor for AccelGyroTempRosPublisher.
   */
  ~AccelGyroTempRosPublisher();

  /**
   * @brief Set the message data provider.
   * @param dataProvider Data provider.
   */
  void setMessageProvider(AccelGyroTempDataProviderInterface * dataProvider) override;

  /**
   * @brief Publish the AccelGyroTempData message.
   */
  void publish() override;

private:
  /*! This variable retains the data provider instance. */
  AccelGyroTempDataProviderInterface * m_data_provider;

  /*! This variable retains the publisher instance. */
  rclcpp::Publisher<imu_ros2::msg::AccelGyroTempData>::SharedPtr m_publisher;

  /*! This variable retains the message that is published. */
  imu_ros2::msg::AccelGyroTempData m_message;
};

#endif  // ACCELGYROTEMP_ROS_SUBSCRIBER_H
