#ifndef IMU_DIAG_ROS_DEVICE_INTERFACE_H
#define IMU_DIAG_ROS_DEVICE_INTERFACE_H

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "imu_ros2/imu_diag_data_provider_interface.h"

class ImuDiagRosDeviceInterface
{
public:
  /**
   * @brief Constructor for ImuDiagRosDeviceInterface.
   */
  ImuDiagRosDeviceInterface(std::shared_ptr<rclcpp::Node> & node);

  /**
   * @brief Destructor for ImuDiagRosDeviceInterface.
   */
  virtual ~ImuDiagRosDeviceInterface();

  virtual void createPublisher() = 0;

  virtual void publishMessage(ImuDiagDataProviderInterface * dataProvider) = 0;

protected:
  /*! The ros2 Node data member. */
  std::shared_ptr<rclcpp::Node> m_node;
};

#endif // IMU_DIAG_ROS_DEVICE_INTERFACE_H
