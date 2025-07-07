/*******************************************************************************
 *   @file   imu_diag_ros_publisher.h
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

#ifndef IMU_DIAG_ROS_PUBLISHER_H
#define IMU_DIAG_ROS_PUBLISHER_H

#include <rclcpp/rclcpp.hpp>

#include "adi_imu/imu_diag_data_provider_interface.h"
#include "adi_imu/imu_diag_ros_publisher_interface.h"

namespace adi_imu
{

/**
 * @brief Class for diagnosis publisher for adis1657x chips.
 */
class ImuDiagRosPublisher : public ImuDiagRosPublisherInterface
{
public:
  /**
   * @brief Constructor for ImuDiagRosPublisher.
   * @param node The ros2 Node instance.
   */
  ImuDiagRosPublisher(std::shared_ptr<rclcpp::Node> & node);

  /**
   * @brief Destructor for ImuDiagRosPublisher.
   */
  ~ImuDiagRosPublisher();

  /**
   * @brief Set the message data provider.
   * @param dataProvider Data provider.
   */
  void setMessageProvider(ImuDiagDataProviderInterface * dataProvider) override;

  /**
   * @brief Set the device descriptor that defines the device's capabilities, register layout
   * and supported features.
   */
  void setDeviceDescriptor(std::shared_ptr<ADISRegisterMap> device_descriptor) override;

  /**
   * @brief Run the thread responsible for publishing ImuDiagData message.
   */
  void run() override;

private:
  /*! This variable retains the data provider instance. */
  ImuDiagDataProviderInterface * m_data_provider;
  std::shared_ptr<ADISRegisterMap> m_device_descriptor;
  std::string m_device_family;


  /*! This variable retains the publisher instance. */
  rclcpp::Publisher<adi_imu::msg::ImuDiagDataADIS1646X>::SharedPtr m_publisher_1646X;
  rclcpp::Publisher<adi_imu::msg::ImuDiagDataADIS1647X>::SharedPtr m_publisher_1647X;
  rclcpp::Publisher<adi_imu::msg::ImuDiagDataADIS1650X>::SharedPtr m_publisher_1650X;
  rclcpp::Publisher<adi_imu::msg::ImuDiagDataADIS1654X>::SharedPtr m_publisher_1654X;
  rclcpp::Publisher<adi_imu::msg::ImuDiagDataADIS1655X>::SharedPtr m_publisher_1655X;
  rclcpp::Publisher<adi_imu::msg::ImuDiagDataADIS1657X>::SharedPtr m_publisher_1657X;

  /*! This variable retains the message that is published. */
  adi_imu::msg::ImuDiagDataADIS1646X m_message_1646X;
  adi_imu::msg::ImuDiagDataADIS1647X m_message_1647X;
  adi_imu::msg::ImuDiagDataADIS1650X m_message_1650X;
  adi_imu::msg::ImuDiagDataADIS1654X m_message_1654X;
  adi_imu::msg::ImuDiagDataADIS1655X m_message_1655X;
  adi_imu::msg::ImuDiagDataADIS1657X m_message_1657X;
};

}  // namespace adi_imu

#endif  // IMU_DIAG_ROS_PUBLISHER_H
