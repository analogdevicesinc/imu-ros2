#include "imu_ros2/imudiag/imu_diag_ros_device_adis1657x.h"

ImuDiagRosDevice_adis1657x::ImuDiagRosDevice_adis1657x(std::shared_ptr<rclcpp::Node> &node) :
  ImuDiagRosDeviceInterface(node)
{
  createPublisher();
}

ImuDiagRosDevice_adis1657x::~ImuDiagRosDevice_adis1657x()
{

}


void ImuDiagRosDevice_adis1657x::createPublisher()
{
  m_publisher_adis1657x = m_node->create_publisher<imu_ros2::msg::ImuDiagDataadis1657x>("imudiagdata", 10);
}

void ImuDiagRosDevice_adis1657x::publishMessage(ImuDiagDataProviderInterface * dataProvider)
{
  if (dataProvider->getData(m_message_adis1657x)) {
    rclcpp::Time now = m_node->get_clock()->now();
    m_message_adis1657x.header.stamp = now;
    m_publisher_adis1657x->publish(m_message_adis1657x);
  } else
    RCLCPP_INFO(rclcpp::get_logger("imu_diag_ros_publisher"), "error reading diagnosis data");

}
