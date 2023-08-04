/*******************************************************************************
 *   @file   velangtemp_ros_publisher.cpp
 *   @brief  Implementation for temperature, delta velocity and delta angle
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
 *******************************************************************************/

#include "imu_ros2/velangtemp_ros_publisher.h"

#include <chrono>
#include <thread>

#include "imu_ros2/setting_declarations.h"

VelAngTempRosPublisher::VelAngTempRosPublisher(std::shared_ptr<rclcpp::Node> & node) { init(node); }

VelAngTempRosPublisher::~VelAngTempRosPublisher() { delete m_data_provider; }

void VelAngTempRosPublisher::init(std::shared_ptr<rclcpp::Node> & node)
{
  m_node = node;
  m_publisher = node->create_publisher<imu_ros2::msg::VelAngTempData>("velangtempdata", 10);
}

void VelAngTempRosPublisher::setMessageProvider(VelAngTempDataProviderInterface * dataProvider)
{
  m_data_provider = dataProvider;
}

bool VelAngTempRosPublisher::configureBufferedDataOutput()
{
  return m_data_provider->configureBufferedDataOutput();
}

void VelAngTempRosPublisher::run()
{
  if (m_data_provider->getData(m_message))
    m_publisher->publish(m_message);
  else
    RCLCPP_INFO(
      rclcpp::get_logger("rclcpp_velangtemp"),
      "error reading delta angle, delta velocity and temperature buffered data");
}
