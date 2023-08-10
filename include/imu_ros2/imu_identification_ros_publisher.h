/*******************************************************************************
 *   @file   imu_identification_ros_publisher.h
 *   @brief  Header for imu identification publisher.
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

#ifndef IMU_IDENTIFICATION_ROS_PUBLISHER_H
#define IMU_IDENTIFICATION_ROS_PUBLISHER_H

#include <rclcpp/rclcpp.hpp>

#include "imu_ros2/imu_identification_data_provider_interface.h"
#include "imu_ros2/imu_identification_ros_publisher_interface.h"

/**
 * @brief Class for identification publisher.
 */
class ImuIdentificationRosPublisher : public ImuIdentificationRosPublisherInterface
{
public:
  /**
   * @brief Constructor for ImuIdentificationRosPublisher.
   * @param node The ros2 Node instance.
   */
  ImuIdentificationRosPublisher(std::shared_ptr<rclcpp::Node> & node);

  /**
   * @brief Destructor for ImuIdentificationRosPublisher.
   */
  ~ImuIdentificationRosPublisher();

  /**
   * @brief Set the message data provider.
   * @param dataProvider Data provider.
   */
  void setMessageProvider(ImuIdentificationDataProviderInterface * dataProvider) override;

  /**
   * @brief Run the thread responsible for publishing ImuIdentificationData
   * message.
   */
  void run() override;

private:
  /*! This variable retains the data provider instance. */
  ImuIdentificationDataProviderInterface * m_data_provider;

  /*! This variable retains the publisher instance. */
  rclcpp::Publisher<imu_ros2::msg::ImuIdentificationData>::SharedPtr m_publisher;

  /*! This variable retains the message that is published. */
  imu_ros2::msg::ImuIdentificationData m_message;
};

#endif  // IMU_IDENTIFICATION_ROS_PUBLISHER_H
