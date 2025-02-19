#include "imu_ros2/imudiag/imu_diag_ros_publisher_devices_interface.h"

ImuDiagRosPublisherDevicesInterface::ImuDiagRosPublisherDevicesInterface(std::shared_ptr<rclcpp::Node> &node) {
  m_node = node;
}

ImuDiagRosPublisherDevicesInterface::~ImuDiagRosPublisherDevicesInterface() { }
