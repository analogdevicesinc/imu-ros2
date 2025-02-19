#include "imu_ros2/imudiag/imu_diag_ros_device_adis1654x.h"

ImuDiagRosDevice_adis1654x::ImuDiagRosDevice_adis1654x(std::shared_ptr<rclcpp::Node> &node) :
  ImuDiagRosDeviceInterface(node)
{
  createPublisher();
}

ImuDiagRosDevice_adis1654x::~ImuDiagRosDevice_adis1654x()
{

}


void ImuDiagRosDevice_adis1654x::createPublisher()
{
  m_publisher_adis1654x = m_node->create_publisher<imu_ros2::msg::ImuDiagDataadis1654x>("imudiagdata", 10);
}

void ImuDiagRosDevice_adis1654x::publishMessage(ImuDiagDataProviderInterface * dataProvider)
{
  if (dataProvider->getData(m_message_adis1654x)) {
    rclcpp::Time now = m_node->get_clock()->now();
    m_message_adis1654x.header.stamp = now;
    m_publisher_adis1654x->publish(m_message_adis1654x);
  } else
    RCLCPP_INFO(rclcpp::get_logger("imu_diag_ros_publisher"), "error reading diagnosis data");

}
