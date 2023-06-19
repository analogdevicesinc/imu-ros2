/***************************************************************************//**
 *   @file   imu_ros_publisher.cpp
 *   @brief  Implementation for standard ros imu data publisher.
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

#include "imu_ros2/imu_ros_publisher.h"
#include "imu_ros2/setting_declarations.h"
#include <thread>
#include <chrono>

ImuRosPublisher::ImuRosPublisher(std::shared_ptr<rclcpp::Node> &node)
{
  init(node);
}

ImuRosPublisher::~ImuRosPublisher()
{
  delete m_data_provider;
}

void ImuRosPublisher::init(std::shared_ptr<rclcpp::Node> &node)
{
  m_node = node;
  m_publisher = node->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
}

void ImuRosPublisher::setMessageProvider(ImuDataProviderInterface *dataProvider)
{
  m_data_provider = dataProvider;
}

void ImuRosPublisher::run()
{
  bool bufferedDataEnabled = false;
  int32_t measuredDataSelection = IMU_STD_MSG_DATA;

  std::thread::id this_id = std::this_thread::get_id();
  std::cout << "thread " << this_id << " started...\n";
  RCLCPP_INFO(rclcpp::get_logger("rclcpp_imuros"), "startThread: '%d'", this_id);

  while (rclcpp::ok())
  {
    measuredDataSelection =
      m_node->get_parameter("measured_data_topic_selection").get_parameter_value().get<int32_t>();

    switch (measuredDataSelection)
    {
    case IMU_STD_MSG_DATA:
      if (!bufferedDataEnabled)
      {
        if (m_data_provider->enableBufferedDataOutput())
          bufferedDataEnabled = true;
      }

      if (m_data_provider->getData(m_message))
        m_publisher->publish(m_message);
      else
        RCLCPP_INFO(rclcpp::get_logger("rclcpp_imuros"), "error reading standard imu buffered data");

      break;

    default:
    {
      bufferedDataEnabled = false;
      break;
    }
    }
  }

  this_id = std::this_thread::get_id();
  std::cout << "thread " << this_id << " ended...\n";
  RCLCPP_INFO(rclcpp::get_logger("rclcpp_imuros"), "endThread: '%d'", this_id);
}
