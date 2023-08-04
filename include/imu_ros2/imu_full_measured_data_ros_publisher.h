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
 * \brief Class for accel, gyro, temp, delta velocity,
 *  delta angle ros publisher.
 *
 * This class initializes the ros Node class.
 * It set message provider with a variable that is
 * a type of ImuFullMeasuredDataProviderInterface.
 * It also run on thread reading from data provider and
 * write on a ros2 publisher.
 */
class ImuFullMeasuredDataRosPublisher : public ImuFullMeasuredDataRosPublisherInterface
{
public:
  /**
   * \brief Constructor for ImuFullMeasuredDataRosPublisher.
   *
   * This is the default constructor for class
   *  ImuFullMeasuredDataRosPublisher.
   *
   * @param node The ros2 Node instance.
   */
  ImuFullMeasuredDataRosPublisher(std::shared_ptr<rclcpp::Node> & node);

  /**
   * \brief Destructor for AccelGyroTempRosPublisher.
   *
   * This is a destructor for AccelGyroTempRosPublisher.
   *
   */
  ~ImuFullMeasuredDataRosPublisher();

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
   * inherit ImuFullMeasuredDataProviderInterface.
   *
   * @param dataProvider Data message provider.
   */
  void setMessageProvider(ImuFullMeasuredDataProviderInterface * dataProvider) override;

  /**
   * \brief Configures the type of data to be written in the buffer
   *
   * This function configures the type of data to be written in the buffer.
   *
   * \return Return true if the configures is successful, false otherwise.
   */
  bool configureBufferedDataOutput() override;

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
  ImuFullMeasuredDataProviderInterface * m_data_provider;

  /*! This variable retain a publisher instance */
  rclcpp::Publisher<imu_ros2::msg::ImuFullMeasuredData>::SharedPtr m_publisher;

  /*! This variable retain a message that is published on a topic */
  imu_ros2::msg::ImuFullMeasuredData m_message;
};

#endif  // IMU_FULL_MEASURED_DATA_ROS_PUBLISHER_H
