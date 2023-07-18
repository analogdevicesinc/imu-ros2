/*******************************************************************************
 *   @file   imu_1657x_diag_ros_publisher.h
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

#ifndef IMU_1657X_DIAG_ROS_PUBLISHER_H
#define IMU_1657X_DIAG_ROS_PUBLISHER_H

#include <rclcpp/rclcpp.hpp>

#include "imu_ros2/imu_1657x_diag_data_provider_interface.h"
#include "imu_ros2/imu_1657x_diag_ros_publisher_interface.h"

/**
 * \brief Class for adis1657x diag ros publisher.
 *
 * This class initializes the ros Node class.
 * It set message provider with a variable that is
 * a type of Imu1657xDiagDataProviderInterface.
 * It also run on thread reading from data provider and
 * write on a ros2 publisher.
 */
class Imu1657xDiagRosPublisher : public Imu1657xDiagRosPublisherInterface
{
public:
  /**
   * \brief Constructor for Imu1657xDiagRosPublisher.
   *
   * This is the default constructor for class
   *  Imu1657xDiagRosPublisher.
   *
   * @param node The ros2 Node instance.
   */
  Imu1657xDiagRosPublisher(std::shared_ptr<rclcpp::Node> & node);

  /**
   * \brief Destructor for Imu1657xDiagRosPublisher.
   *
   * This is a destructor for Imu1657xDiagRosPublisher.
   *
   */
  ~Imu1657xDiagRosPublisher();

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
   * inherit Imu1657xDiagDataProviderInterface.
   *
   * @param dataProvider Data message provider.
   */
  void setMessageProvider(Imu1657xDiagDataProviderInterface * dataProvider) override;

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
  Imu1657xDiagDataProviderInterface * m_data_provider;

  /*! This variable retain a publisher instance */
  rclcpp::Publisher<imu_ros2::msg::Imu1657xDiagData>::SharedPtr m_publisher;

  /*! This variable retain a message that is published on a topic */
  imu_ros2::msg::Imu1657xDiagData m_message;
};

#endif  // IMU_1657X_DIAG_ROS_PUBLISHER_H
