/***************************************************************************//**
 *   @file   imu_16505_diag_ros_publisher.cpp
 *   @brief  Implementation for adis16505 diagnosis publisher.
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

#include "imu_ros2/imu_16505_diag_ros_publisher.h"
#include <thread>

Imu16505DiagRosPublisher::Imu16505DiagRosPublisher(std::shared_ptr<rclcpp::Node> &node)
{
  init(node);
}

Imu16505DiagRosPublisher::~Imu16505DiagRosPublisher()
{
  delete m_data_provider;
}

void Imu16505DiagRosPublisher::init(std::shared_ptr<rclcpp::Node> &node)
{
  m_node = node;
  m_publisher = node->create_publisher<imu_ros2::msg::Imu16505DiagData>("Imu16505DiagData", 10);
}

void Imu16505DiagRosPublisher::setMessageProvider(Imu16505DiagDataProviderInterface *dataProvider)
{
  m_data_provider = dataProvider;
}

void Imu16505DiagRosPublisher::run()
{
  std::thread::id this_id = std::this_thread::get_id();
  std::cout << "thread " << this_id << " started...\n";
  RCLCPP_INFO(rclcpp::get_logger("rclcpp_imu16505diag"), "startThread: '%d'", this_id);

  while (rclcpp::ok())
  {
    if (m_data_provider->getData(m_message))
      m_publisher->publish(m_message);
    else
      RCLCPP_INFO(rclcpp::get_logger("rclcpp_imu16505diag"), "error reading diagnosis data");
  }

  this_id = std::this_thread::get_id();
  std::cout << "thread " << this_id << " ended...\n";
  RCLCPP_INFO(rclcpp::get_logger("rclcpp_imu16505diag"), "endThread: '%d'", this_id);
}
