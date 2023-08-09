/*******************************************************************************
 *   @file   accelgyrotemp_ros_publisher.cpp
 *   @brief  Implementation for acceleration, gyroscope and temperature
 *           publisher.
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

#include "imu_ros2/accelgyrotemp_ros_publisher.h"

#include <chrono>
#include <thread>

#include "imu_ros2/setting_declarations.h"

AccelGyroTempRosPublisher::AccelGyroTempRosPublisher(std::shared_ptr<rclcpp::Node> & node)
{
  init(node);
}

AccelGyroTempRosPublisher::~AccelGyroTempRosPublisher() { delete m_data_provider; }

void AccelGyroTempRosPublisher::init(std::shared_ptr<rclcpp::Node> & node)
{
  m_node = node;
  m_publisher = node->create_publisher<imu_ros2::msg::AccelGyroTempData>("accelgyrotempdata", 10);
}

void AccelGyroTempRosPublisher::setMessageProvider(
  AccelGyroTempDataProviderInterface * dataProvider)
{
  m_data_provider = dataProvider;
}

void AccelGyroTempRosPublisher::run()
{
  if (m_data_provider->getData(m_message))
    m_publisher->publish(m_message);
  else
    RCLCPP_INFO(
      rclcpp::get_logger("rclcpp_accelgyrotemp"),
      "error reading accelerometer, gyroscope and temperature buffered data");
}
