/*******************************************************************************
 *   @file   imu_ros_publisher.h
 *   @brief  Header for IMU ros standard message publisher.
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

#ifndef IMU_ROS_PUBLISHER_H
#define IMU_ROS_PUBLISHER_H

#include <rclcpp/rclcpp.hpp>

#include "imu_ros2/imu_data_provider_interface.h"
#include "imu_ros2/imu_ros_publisher_interface.h"

/**
 * \brief Class for accel and gyro ros publisher.
 *
 * This class initializes the ros Node class.
 * It set message provider with a variable that is
 * a standard type of sensor_msgs::msg::Imu.
 * It also run on thread reading from data provider and
 * write on a ros2 publisher.
 */
class ImuRosPublisher : public ImuRosPublisherInterface
{
public:
  /**
   * \brief Constructor for ImuRosPublisher.
   *
   * This is the default constructor for class
   *  ImuRosPublisher.
   *
   * @param node The ros2 Node instance.
   */
  ImuRosPublisher(std::shared_ptr<rclcpp::Node> & node);

  /**
   * \brief Destructor for ImuRosPublisher.
   *
   * This is a destructor for ImuRosPublisher.
   *
   */
  ~ImuRosPublisher();

  /**
   * @brief Initialize class with ros2 Node instance.
   *
   * This function initialize the class that inherit
   * this interface wiht a ros2 Node instance.
   *
   * @param node The ros2 Node instance.
   */
  void init(std::shared_ptr<rclcpp::Node> & node) override;

  /**
   * @brief Set message provider.
   *
   * This function set data message provider with a variable that
   * inherit AccelGyroTempDataProviderInterface.
   *
   * @param dataProvider Data message provider.
   */
  void setMessageProvider(ImuDataProviderInterface * dataProvider) override;

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
  ImuDataProviderInterface * m_data_provider;

  /*! This variable retain a publisher instance */
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_publisher;

  /*! This variable retain a message that is published on a topic */
  sensor_msgs::msg::Imu m_message;
};

#endif  // IMU_ROS_PUBLISHER_H
