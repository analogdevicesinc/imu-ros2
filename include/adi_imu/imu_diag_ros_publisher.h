/*******************************************************************************
 *   @file   imu_diag_ros_publisher.h
 *   @brief  Header for adis1657x diagnosis publisher.
 *   @author Vasile Holonec (Vasile.Holonec@analog.com)
*******************************************************************************/
// Copyright 2023 Analog Devices, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ADI_IMU__IMU_DIAG_ROS_PUBLISHER_H_
#define ADI_IMU__IMU_DIAG_ROS_PUBLISHER_H_

#include <memory>
#include <string>

#include "adi_imu/imu_diag_data_provider_interface.h"
#include "adi_imu/imu_diag_ros_publisher_interface.h"
#include "rclcpp/rclcpp.hpp"

namespace adi_imu
{

/**
 * @brief Class for diagnosis publisher for adis1657x chips.
 */
template <typename DiagMsgType>
class ImuDiagRosPublisher : public ImuDiagRosPublisherInterface
{
public:
  /**
   * @brief Constructor for ImuDiagRosPublisher.
   * @param node The ros2 Node instance.
   */
  explicit ImuDiagRosPublisher(std::shared_ptr<rclcpp::Node> & node)
  {
    m_node = node;
    m_publisher = m_node->create_publisher<DiagMsgType>("imudiagdata", 10);
  }

  /**
   * @brief Destructor for ImuDiagRosPublisher.
   */
  ~ImuDiagRosPublisher() { delete m_data_provider; }

  /**
   * @brief Set the message data provider.
   * @param dataProvider Data provider.
   */
  void setMessageProvider(ImuDiagDataProviderInterface * dataProvider) override
  {
    m_data_provider = dataProvider;
  }

  /**
   * @brief Set the device descriptor that defines the device's capabilities, register layout
   * and supported features.
   */
  void setDeviceDescriptor(std::shared_ptr<ADISRegisterMap> device_descriptor) override
  {
    m_device_descriptor = device_descriptor;
    m_device_family = m_device_descriptor->getDeviceFamily();
  }

  /**
   * @brief Run the thread responsible for publishing ImuDiagData message.
   */
  void run() override
  {
    std::thread::id this_id = std::this_thread::get_id();
    std::cout << "thread " << this_id << " started...\n";
    RCLCPP_INFO(rclcpp::get_logger("imu_diag_ros_publisher"), "startThread: ImuDiagRosPublisher");

    while (rclcpp::ok()) {
      if (m_publisher && m_data_provider && m_data_provider->getData(m_message)) {
        rclcpp::Time now = m_node->get_clock()->now();
        m_message.header.stamp = now;
        m_publisher->publish(m_message);
      } else {
        RCLCPP_INFO(rclcpp::get_logger("imu_diag_ros_publisher"), "error reading diagnosis data");
      }
    }

    this_id = std::this_thread::get_id();
    std::cout << "thread " << this_id << " ended...\n";
    RCLCPP_INFO(rclcpp::get_logger("imu_diag_ros_publisher"), "endThread: ImuDiagRosPublisher");
  }

private:
  /*! This variable retains the data provider instance. */
  ImuDiagDataProviderInterface * m_data_provider;
  std::shared_ptr<ADISRegisterMap> m_device_descriptor;
  std::string m_device_family;

  /*! This variable retains the publisher instance. */
  typename rclcpp::Publisher<DiagMsgType>::SharedPtr m_publisher;

  /*! This variable retains the message that is published. */
  DiagMsgType m_message;
};

}  // namespace adi_imu

#endif  // ADI_IMU__IMU_DIAG_ROS_PUBLISHER_H_
