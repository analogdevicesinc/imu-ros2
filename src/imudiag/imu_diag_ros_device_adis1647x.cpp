#include "imu_ros2/imudiag/imu_diag_ros_device_adis1647x.h"

ImuDiagRosDevice_adis1647x::ImuDiagRosDevice_adis1647x(std::shared_ptr<rclcpp::Node> &node) :
  ImuDiagRosDeviceInterface(node)
{
  createPublisher();
}

ImuDiagRosDevice_adis1647x::~ImuDiagRosDevice_adis1647x()
{

}


void ImuDiagRosDevice_adis1647x::createPublisher()
{
  m_publisher_adis1647x = m_node->create_publisher<imu_ros2::msg::ImuDiagDataadis1647x>("imudiagdata", 10);
}

void ImuDiagRosDevice_adis1647x::publishMessage(ImuDiagDataProviderInterface * dataProvider)
{
  if (dataProvider->getData(m_message_adis1647x)) {
    rclcpp::Time now = m_node->get_clock()->now();
    m_message_adis1647x.header.stamp = now;
    m_publisher_adis1647x->publish(m_message_adis1647x);
  } else
    RCLCPP_INFO(rclcpp::get_logger("imu_diag_ros_publisher"), "error reading diagnosis data");

}
