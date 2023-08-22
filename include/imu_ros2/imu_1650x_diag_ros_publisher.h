/*******************************************************************************
 *   @file   imu_1650x_diag_ros_publisher.h
 *   @brief  Header for adis1650x diagnosis publisher.
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

#ifndef IMU_1650X_DIAG_ROS_SUBSCRIBER_H
#define IMU_1650X_DIAG_ROS_SUBSCRIBER_H

#include <rclcpp/rclcpp.hpp>

#include "imu_ros2/imu_1650x_diag_data_provider_interface.h"
#include "imu_ros2/imu_1650x_diag_ros_publisher_interface.h"

/**
 * @brief Class for diagnosis publisher for adis1650x chips.
 */
class Imu1650xDiagRosPublisher : public Imu1650xDiagRosPublisherInterface
{
public:
  /**
   * @brief Constructor for Imu1650xDiagRosPublisher.
   * @param node The ros2 Node instance.
   */
  Imu1650xDiagRosPublisher(std::shared_ptr<rclcpp::Node> & node);

  /**
   * @brief Destructor for Imu1650xDiagRosPublisher.
   */
  ~Imu1650xDiagRosPublisher();

  /**
   * @brief Set the message data provider.
   * @param dataProvider Data provider.
   */
  void setMessageProvider(Imu1650xDiagDataProviderInterface * dataProvider) override;

  /**
   * @brief Run the thread responsible for publishing Imu1650xDiagData message.
   */
  void run() override;

private:
  /*! This variable retains the data provider instance. */
  Imu1650xDiagDataProviderInterface * m_data_provider;

  /*! This variable retains the publisher instance. */
  rclcpp::Publisher<imu_ros2::msg::Imu1650xDiagData>::SharedPtr m_publisher;

  /*! This variable retains the message that is published. */
  imu_ros2::msg::Imu1650xDiagData m_message;
};

#endif  // IMU_1650X_DIAG_ROS_SUBSCRIBER_H
