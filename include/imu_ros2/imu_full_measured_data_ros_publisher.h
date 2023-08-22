/*******************************************************************************
 *   @file   imu_full_measured_data_ros_publisher.h
 *   @brief  Header for acceleration, gyroscope, temperature, delta velocity,
 *           delta angle and temperature publisher.
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

#ifndef IMU_FULL_MEASURED_DATA_ROS_PUBLISHER_H
#define IMU_FULL_MEASURED_DATA_ROS_PUBLISHER_H

#include <rclcpp/rclcpp.hpp>

#include "imu_ros2/imu_full_measured_data_provider_interface.h"
#include "imu_ros2/imu_full_measured_data_ros_publisher_interface.h"

/**
 * @brief Class for acceleration, angular velocity, delta angle, delta velocity
 * and temperature publisher.
 */
class ImuFullMeasuredDataRosPublisher : public ImuFullMeasuredDataRosPublisherInterface
{
public:
  /**
   * @brief Constructor for ImuFullMeasuredDataRosPublisher.
   * @param node The ros2 Node instance.
   */
  ImuFullMeasuredDataRosPublisher(std::shared_ptr<rclcpp::Node> & node);

  /**
   * @brief Destructor for AccelGyroTempRosPublisher.
   */
  ~ImuFullMeasuredDataRosPublisher();

  /**
   * @brief Set the message data provider.
   * @param dataProvider Data provider.
   */
  void setMessageProvider(ImuFullMeasuredDataProviderInterface * dataProvider) override;

  /**
   * @brief Publish the ImuFullMeasuredData message.
   */
  void publish() override;

private:
  /*! This variable retains the data provider instance. */
  ImuFullMeasuredDataProviderInterface * m_data_provider;

  /*! This variable retains the publisher instance. */
  rclcpp::Publisher<imu_ros2::msg::ImuFullMeasuredData>::SharedPtr m_publisher;

  /*! This variable retains the message that is published. */
  imu_ros2::msg::ImuFullMeasuredData m_message;
};

#endif  // IMU_FULL_MEASURED_DATA_ROS_PUBLISHER_H
