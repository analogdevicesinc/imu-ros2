/*******************************************************************************
 *   @file   imu_diag_ros_publisher.cpp
 *   @brief  Implementation for adis diagnosis publisher.
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

#include "adi_imu/imu_diag_ros_publisher.h"

#include <thread>

namespace adi_imu
{

ImuDiagRosPublisher::ImuDiagRosPublisher(std::shared_ptr<rclcpp::Node> & node)
{
  m_node = node;

  m_publisher_1646X = nullptr;
  m_publisher_1647X = nullptr;
  m_publisher_1650X = nullptr;
  m_publisher_1654X = nullptr;
  m_publisher_1655X = nullptr;
  m_publisher_1657X = nullptr;
}

ImuDiagRosPublisher::~ImuDiagRosPublisher() { delete m_data_provider; }

void ImuDiagRosPublisher::setMessageProvider(ImuDiagDataProviderInterface * dataProvider)
{
  m_data_provider = dataProvider;
}

void ImuDiagRosPublisher::setDeviceDescriptor(std::shared_ptr<ADISRegisterMap> device_descriptor)
{
  m_device_descriptor = device_descriptor;
  m_device_family = m_device_descriptor->getDeviceFamily();

  auto topic_name = "imudiagdata";
  if (m_device_family == "adis1646x") {
    m_publisher_1646X = m_node->create_publisher<adi_imu::msg::ImuDiagDataADIS1646X>(topic_name, 10);
  } else if (m_device_family == "adis1647x") {
    m_publisher_1647X = m_node->create_publisher<adi_imu::msg::ImuDiagDataADIS1647X>(topic_name, 10);
  } else if (m_device_family == "adis1650x") {
    m_publisher_1650X = m_node->create_publisher<adi_imu::msg::ImuDiagDataADIS1650X>(topic_name, 10);
  } else if (m_device_family == "adis1654x") {
    m_publisher_1654X = m_node->create_publisher<adi_imu::msg::ImuDiagDataADIS1654X>(topic_name, 10);
  } else if (m_device_family == "adis1655x") {
    m_publisher_1655X = m_node->create_publisher<adi_imu::msg::ImuDiagDataADIS1655X>(topic_name, 10);
  } else if (m_device_family == "adis1657x") {
    m_publisher_1657X = m_node->create_publisher<adi_imu::msg::ImuDiagDataADIS1657X>(topic_name, 10);
  }
}

void ImuDiagRosPublisher::run()
{
  std::thread::id this_id = std::this_thread::get_id();
  std::cout << "thread " << this_id << " started...\n";
  RCLCPP_INFO(rclcpp::get_logger("imu_diag_ros_publisher"), "startThread: ImuDiagRosPublisher");

  while (rclcpp::ok()) {
    if (m_publisher_1646X && m_data_provider->getData(m_message_1646X)) {
      rclcpp::Time now = m_node->get_clock()->now();
      m_message_1646X.header.stamp = now;
      m_publisher_1646X->publish(m_message_1646X);
    } else if (m_publisher_1647X && m_data_provider->getData(m_message_1647X)) {
      rclcpp::Time now = m_node->get_clock()->now();
      m_message_1647X.header.stamp = now;
      m_publisher_1647X->publish(m_message_1647X);
    } else if (m_publisher_1650X && m_data_provider->getData(m_message_1650X)) {
      rclcpp::Time now = m_node->get_clock()->now();
      m_message_1650X.header.stamp = now;
      m_publisher_1650X->publish(m_message_1650X);
    } else if (m_publisher_1654X && m_data_provider->getData(m_message_1654X)) {
      rclcpp::Time now = m_node->get_clock()->now();
      m_message_1654X.header.stamp = now;
      m_publisher_1654X->publish(m_message_1654X);
    } else if (m_publisher_1655X && m_data_provider->getData(m_message_1655X)) {
      rclcpp::Time now = m_node->get_clock()->now();
      m_message_1655X.header.stamp = now;
      m_publisher_1655X->publish(m_message_1655X);
    } else if (m_publisher_1657X && m_data_provider->getData(m_message_1657X)) {
      rclcpp::Time now = m_node->get_clock()->now();
      m_message_1657X.header.stamp = now;
      m_publisher_1657X->publish(m_message_1657X);
    }
    else {
      RCLCPP_INFO(rclcpp::get_logger("imu_diag_ros_publisher"), "error reading diagnosis data");
    }
  }

  this_id = std::this_thread::get_id();
  std::cout << "thread " << this_id << " ended...\n";
  RCLCPP_INFO(rclcpp::get_logger("imu_diag_ros_publisher"), "endThread: ImuDiagRosPublisher");
}

}  // namespace adi_imu