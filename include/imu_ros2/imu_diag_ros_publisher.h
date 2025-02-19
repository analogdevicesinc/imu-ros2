/*******************************************************************************
 *   @file   imu_diag_ros_publisher.h
 *   @brief  Header for adis1657x diagnosis publisher.
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

#ifndef IMU_DIAG_ROS_PUBLISHER_H
#define IMU_DIAG_ROS_PUBLISHER_H

#include <rclcpp/rclcpp.hpp>

#include "imu_diag_data_provider_interface.h"
#include "imu_diag_ros_publisher_interface.h"
#include "imudiag/imu_diag_ros_publisher_devices.h"

/**
 * @brief Class for diagnosis publisher for adis1657x chips.
 */
class ImuDiagRosPublisher : public ImuDiagRosPublisherInterface
{
public:
  /**
   * @brief Constructor for ImuDiagRosPublisher.
   * @param node The ros2 Node instance.
   */
  ImuDiagRosPublisher(std::shared_ptr<rclcpp::Node> & node);

  /**
   * @brief Destructor for ImuDiagRosPublisher.
   */
  ~ImuDiagRosPublisher();

  /**
   * @brief Set the message data provider.
   * @param dataProvider Data provider.
   */
  void setMessageProvider(ImuDiagDataProviderInterface * dataProvider) override;

  /**
   * @brief Run the thread responsible for publishing ImuDiagData message.
   */
  void run() override;

private:
  /*! This variable retains the data provider instance. */
  ImuDiagDataProviderInterface * m_data_provider;

  /*! This variable retains the publisher instance. */
  ImuDiagRosPublisherDevices * m_diag_publisher_device;
};

#endif  // IMU_DIAG_ROS_PUBLISHER_H
