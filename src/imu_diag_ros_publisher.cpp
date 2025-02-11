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

#include "imu_ros2/imu_diag_ros_publisher.h"
#include "imu_ros2/adis_data.h"

#include <thread>

ImuDiagRosPublisher::ImuDiagRosPublisher(std::shared_ptr<rclcpp::Node> & node)
{
  m_node = node;
  //m_publisher = node->create_publisher<imu_ros2::msg::ImuDiagData>("imudiagdata", 10);
  if(AdisData::GetInstance()->checkDeviceName("adis1646x"))
    m_publisher_adis1646x = node->create_publisher<imu_ros2::msg::ImuDiagDataadis1646x>("imudiagdata", 10);
  if(AdisData::GetInstance()->checkDeviceName("adis1647x"))
    m_publisher_adis1647x = node->create_publisher<imu_ros2::msg::ImuDiagDataadis1647x>("imudiagdata", 10);
  if(AdisData::GetInstance()->checkDeviceName("adis1650x"))
    m_publisher_adis1650x = node->create_publisher<imu_ros2::msg::ImuDiagDataadis1650x>("imudiagdata", 10);
  if(AdisData::GetInstance()->checkDeviceName("adis1654x"))
    m_publisher_adis1654x = node->create_publisher<imu_ros2::msg::ImuDiagDataadis1654x>("imudiagdata", 10);
  if(AdisData::GetInstance()->checkDeviceName("adis1655x"))
    m_publisher_adis1655x = node->create_publisher<imu_ros2::msg::ImuDiagDataadis1655x>("imudiagdata", 10);
  if(AdisData::GetInstance()->checkDeviceName("adis1657x"))
    m_publisher_adis1657x = node->create_publisher<imu_ros2::msg::ImuDiagDataadis1657x>("imudiagdata", 10);
}

ImuDiagRosPublisher::~ImuDiagRosPublisher() { delete m_data_provider; }

void ImuDiagRosPublisher::setMessageProvider(ImuDiagDataProviderInterface * dataProvider)
{
  m_data_provider = dataProvider;
}

void ImuDiagRosPublisher::run()
{
  std::thread::id this_id = std::this_thread::get_id();
  std::cout << "thread " << this_id << " started...\n";
  RCLCPP_INFO(rclcpp::get_logger("imu_diag_ros_publisher"), "startThread: ImuDiagRosPublisher");

  while (rclcpp::ok()) {
    if(AdisData::GetInstance()->checkDeviceName("adis1646x")) {
    if (m_data_provider->getData(m_message_adis1646x)) {
      rclcpp::Time now = m_node->get_clock()->now();
      m_message_adis1646x.header.stamp = now;
      m_publisher_adis1646x->publish(m_message_adis1646x);
    } else
      RCLCPP_INFO(rclcpp::get_logger("imu_diag_ros_publisher"), "error reading diagnosis data");
    }
    if(AdisData::GetInstance()->checkDeviceName("adis1647x")) {
    if (m_data_provider->getData(m_message_adis1647x)) {
      rclcpp::Time now = m_node->get_clock()->now();
      m_message_adis1647x.header.stamp = now;
      m_publisher_adis1647x->publish(m_message_adis1647x);
    } else
      RCLCPP_INFO(rclcpp::get_logger("imu_diag_ros_publisher"), "error reading diagnosis data");
    }
    if(AdisData::GetInstance()->checkDeviceName("adis1650x")) {
    if (m_data_provider->getData(m_message_adis1650x)) {
      rclcpp::Time now = m_node->get_clock()->now();
      m_message_adis1650x.header.stamp = now;
      m_publisher_adis1650x->publish(m_message_adis1650x);
    } else
      RCLCPP_INFO(rclcpp::get_logger("imu_diag_ros_publisher"), "error reading diagnosis data");
    }
    if(AdisData::GetInstance()->checkDeviceName("adis1654x")) {
    if (m_data_provider->getData(m_message_adis1654x)) {
      rclcpp::Time now = m_node->get_clock()->now();
      m_message_adis1654x.header.stamp = now;
      m_publisher_adis1654x->publish(m_message_adis1654x);
    } else
      RCLCPP_INFO(rclcpp::get_logger("imu_diag_ros_publisher"), "error reading diagnosis data");
    }
    if(AdisData::GetInstance()->checkDeviceName("adis1655x")) {
    if (m_data_provider->getData(m_message_adis1655x)) {
      rclcpp::Time now = m_node->get_clock()->now();
      m_message_adis1655x.header.stamp = now;
      m_publisher_adis1655x->publish(m_message_adis1655x);
    } else
      RCLCPP_INFO(rclcpp::get_logger("imu_diag_ros_publisher"), "error reading diagnosis data");
    }
    if(AdisData::GetInstance()->checkDeviceName("adis1657x")) {
    if (m_data_provider->getData(m_message_adis1657x)) {
      rclcpp::Time now = m_node->get_clock()->now();
      m_message_adis1657x.header.stamp = now;
      m_publisher_adis1657x->publish(m_message_adis1657x);
    } else
      RCLCPP_INFO(rclcpp::get_logger("imu_diag_ros_publisher"), "error reading diagnosis data");
    }
  }

  this_id = std::this_thread::get_id();
  std::cout << "thread " << this_id << " ended...\n";
  RCLCPP_INFO(rclcpp::get_logger("imu_diag_ros_publisher"), "endThread: ImuDiagRosPublisher");
}
