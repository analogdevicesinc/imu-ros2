/*******************************************************************************
 *   @file   velangtemp_ros_publisher.h
 *   @brief  Header temperature, delta velocity and delta angle publisher.
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

#ifndef VELANGTEMP_ROS_PUBLISHER_H
#define VELANGTEMP_ROS_PUBLISHER_H

#include <rclcpp/rclcpp.hpp>

#include "imu_ros2/velangtemp_data_provider_interface.h"
#include "imu_ros2/velangtemp_ros_publisher_interface.h"

/**
 * @brief Class for delta velocity, delta angle and temperature publisher.
 */
class VelAngTempRosPublisher : public VelAngTempRosPublisherInterface
{
public:
  /**
   * @brief Constructor for VelAngTempRosPublisher.
   * @param node The ros2 Node instance.
   */
  VelAngTempRosPublisher(std::shared_ptr<rclcpp::Node> & node);

  /**
   * @brief Destructor for VelAngTempRosPublisher.
   */
  ~VelAngTempRosPublisher();

  /**
   * @brief Set the message data provider.
   * @param dataProvider Data provider.
   */
  void setMessageProvider(VelAngTempDataProviderInterface * dataProvider) override;

  /**
   * @brief Publish the VelAngTempData message.
   */
  void publish() override;

private:
  /*! This variable retains the data provider instance. */
  VelAngTempDataProviderInterface * m_data_provider;

  /*! This variable retains the publisher instance. */
  rclcpp::Publisher<imu_ros2::msg::VelAngTempData>::SharedPtr m_publisher;

  /*! This variable retains the message that is published. */
  imu_ros2::msg::VelAngTempData m_message;
};

#endif  // VELANGTEMP_ROS_PUBLISHER_H
