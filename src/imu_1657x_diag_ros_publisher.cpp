/*******************************************************************************
 *   @file   imu_1657x_diag_ros_publisher.cpp
 *   @brief  Implementation for adis1657x diagnosis publisher.
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

#include "imu_ros2/imu_1657x_diag_ros_publisher.h"

#include <thread>

Imu1657xDiagRosPublisher::Imu1657xDiagRosPublisher(std::shared_ptr<rclcpp::Node> & node)
{
  m_node = node;
  m_publisher = node->create_publisher<imu_ros2::msg::Imu1657xDiagData>("imu1657xdiagdata", 10);
}

Imu1657xDiagRosPublisher::~Imu1657xDiagRosPublisher() { delete m_data_provider; }

void Imu1657xDiagRosPublisher::setMessageProvider(Imu1657xDiagDataProviderInterface * dataProvider)
{
  m_data_provider = dataProvider;
}

void Imu1657xDiagRosPublisher::run()
{
  std::thread::id this_id = std::this_thread::get_id();
  std::cout << "thread " << this_id << " started...\n";
  RCLCPP_INFO(
    rclcpp::get_logger("imu_1657x_diag_ros_publisher"), "startThread: Imu1657xDiagRosPublisher");

  while (rclcpp::ok()) {
    if (m_data_provider->getData(m_message))
      m_publisher->publish(m_message);
    else
      RCLCPP_INFO(
        rclcpp::get_logger("imu_1657x_diag_ros_publisher"), "error reading diagnosis data");
  }

  this_id = std::this_thread::get_id();
  std::cout << "thread " << this_id << " ended...\n";
  RCLCPP_INFO(
    rclcpp::get_logger("imu_1657x_diag_ros_publisher"), "endThread: Imu1657xDiagRosPublisher");
}
