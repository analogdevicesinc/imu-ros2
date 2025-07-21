#ifndef IMU_DIAG_ROS_PUBLISHER_FACTORY_H
#define IMU_DIAG_ROS_PUBLISHER_FACTORY_H

#include <memory>
#include <string>
#include "adi_imu/imu_diag_ros_publisher.h"
#include "adi_imu/adis_register_map.h"

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
  static std::unique_ptr<ImuDiagRosPublisherInterface> make(std::shared_ptr<ADISRegisterMap> device_descriptor,
                                                            std::shared_ptr<rclcpp::Node>& node)
  {
    std::string family = device_descriptor->getDeviceFamily();

    if (family == "adis1646x")
    {
      return std::make_unique<ImuDiagRosPublisher<adi_imu::msg::ImuDiagDataADIS1646X>>(node);
    }
    else if (family == "adis1647x")
    {
      return std::make_unique<ImuDiagRosPublisher<adi_imu::msg::ImuDiagDataADIS1647X>>(node);
    }
    else if (family == "adis1650x")
    {
      return std::make_unique<ImuDiagRosPublisher<adi_imu::msg::ImuDiagDataADIS1650X>>(node);
    }
    else if (family == "adis1654x")
    {
      return std::make_unique<ImuDiagRosPublisher<adi_imu::msg::ImuDiagDataADIS1654X>>(node);
    }
    else if (family == "adis1655x")
    {
      return std::make_unique<ImuDiagRosPublisher<adi_imu::msg::ImuDiagDataADIS1655X>>(node);
    }
    else if (family == "adis1657x")
    {
      return std::make_unique<ImuDiagRosPublisher<adi_imu::msg::ImuDiagDataADIS1657X>>(node);
    }
    else
    {
      throw std::invalid_argument("Unsupported device family: " + family);
    }
  }
};

}  // namespace adi_imu

#endif  // IMU_DIAG_ROS_PUBLISHER_FACTORY_H
