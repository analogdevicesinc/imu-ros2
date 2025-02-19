#include "imu_ros2/imudiag/imu_diag_ros_device_adis1650x.h"

ImuDiagRosDevice_adis1650x::ImuDiagRosDevice_adis1650x(std::shared_ptr<rclcpp::Node> &node) :
  ImuDiagRosDeviceInterface(node)
{
  createPublisher();
}

ImuDiagRosDevice_adis1650x::~ImuDiagRosDevice_adis1650x()
{

}


void ImuDiagRosDevice_adis1650x::createPublisher()
{
  m_publisher_adis1650x = m_node->create_publisher<imu_ros2::msg::ImuDiagDataadis1650x>("imudiagdata", 10);
}

void ImuDiagRosDevice_adis1650x::publishMessage(ImuDiagDataProviderInterface * dataProvider)
{
  if (dataProvider->getData(m_message_adis1650x)) {
    rclcpp::Time now = m_node->get_clock()->now();
    m_message_adis1650x.header.stamp = now;
    m_publisher_adis1650x->publish(m_message_adis1650x);
  } else
    RCLCPP_INFO(rclcpp::get_logger("imu_diag_ros_publisher"), "error reading diagnosis data");

}
