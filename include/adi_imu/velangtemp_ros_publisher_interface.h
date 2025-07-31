/*******************************************************************************
 *   @file   velangletemp_ros_publisher_interface.h
 *   @brief  Interface temperature, delta velocity and delta angle publisher.
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

#ifndef ADI_IMU__VELANGTEMP_ROS_PUBLISHER_INTERFACE_H_
#define ADI_IMU__VELANGTEMP_ROS_PUBLISHER_INTERFACE_H_

#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace adi_imu
{

class VelAngTempDataProviderInterface;

/**
 * @brief Interface for delta velocity, delta angle and temperature publisher.
 */
class VelAngTempRosPublisherInterface
{
public:
  /**
   * @brief Constructor for VelAngTempRosPublisherInterface.
   */
  VelAngTempRosPublisherInterface() {}

  /**
   * @brief Destructor for VelAngTempRosPublisherInterface.
   */
  virtual ~VelAngTempRosPublisherInterface() {}

  /**
   * @brief Set the message data provider.
   * @param dataProvider Data provider.
   */
  virtual void setMessageProvider(VelAngTempDataProviderInterface * dataProvider) = 0;

  /**
   * @brief Publish the VelAngTempData message.
   */
  virtual void publish() = 0;

protected:
  /*! The ros2 Node data member. */
  std::shared_ptr<rclcpp::Node> m_node;
};

}  // namespace adi_imu

#endif  // ADI_IMU__VELANGTEMP_ROS_PUBLISHER_INTERFACE_H_
