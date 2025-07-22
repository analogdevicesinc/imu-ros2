// Copyright 2025 Analog Devices, Inc.
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

#ifndef ADI_IMU__IMU_DIAG_ROS_PUBLISHER_FACTORY_H_
#define ADI_IMU__IMU_DIAG_ROS_PUBLISHER_FACTORY_H_

#include <memory>
#include <string>

#include "adi_imu/adis_register_map.h"
#include "adi_imu/imu_diag_ros_publisher.h"

// Include all message types
#include "adi_imu/msg/imu_diag_data_adis1646_x.hpp"
#include "adi_imu/msg/imu_diag_data_adis1647_x.hpp"
#include "adi_imu/msg/imu_diag_data_adis1650_x.hpp"
#include "adi_imu/msg/imu_diag_data_adis1654_x.hpp"
#include "adi_imu/msg/imu_diag_data_adis1655_x.hpp"
#include "adi_imu/msg/imu_diag_data_adis1657_x.hpp"

namespace adi_imu
{

class ImuDiagPublisherFactory
{
public:
  static std::unique_ptr<ImuDiagRosPublisherInterface> make(
    std::shared_ptr<ADISRegisterMap> device_descriptor, std::shared_ptr<rclcpp::Node> & node)
  {
    std::string family = device_descriptor->getDeviceFamily();

    if (family == "adis1646x") {
      return std::make_unique<ImuDiagRosPublisher<adi_imu::msg::ImuDiagDataADIS1646X>>(node);
    } else if (family == "adis1647x") {
      return std::make_unique<ImuDiagRosPublisher<adi_imu::msg::ImuDiagDataADIS1647X>>(node);
    } else if (family == "adis1650x") {
      return std::make_unique<ImuDiagRosPublisher<adi_imu::msg::ImuDiagDataADIS1650X>>(node);
    } else if (family == "adis1654x") {
      return std::make_unique<ImuDiagRosPublisher<adi_imu::msg::ImuDiagDataADIS1654X>>(node);
    } else if (family == "adis1655x") {
      return std::make_unique<ImuDiagRosPublisher<adi_imu::msg::ImuDiagDataADIS1655X>>(node);
    } else if (family == "adis1657x") {
      return std::make_unique<ImuDiagRosPublisher<adi_imu::msg::ImuDiagDataADIS1657X>>(node);
    } else {
      throw std::invalid_argument("Unsupported device family: " + family);
    }
  }
};

}  // namespace adi_imu

#endif  // ADI_IMU__IMU_DIAG_ROS_PUBLISHER_FACTORY_H_
