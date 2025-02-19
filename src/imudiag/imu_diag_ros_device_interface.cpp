#include "imu_ros2/imudiag/imu_diag_ros_device_interface.h"

ImuDiagRosDeviceInterface::ImuDiagRosDeviceInterface(std::shared_ptr<rclcpp::Node> & node)
{
  m_node = node;
}

ImuDiagRosDeviceInterface::~ImuDiagRosDeviceInterface()
{

}
