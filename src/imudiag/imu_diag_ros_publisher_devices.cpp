#include "imu_ros2/imudiag/imu_diag_ros_publisher_devices.h"
#include "imu_ros2/adis_data.h"
#include "imu_ros2/imudiag/imu_diag_ros_device_adis1646x.h"
#include "imu_ros2/imudiag/imu_diag_ros_device_adis1647x.h"
#include "imu_ros2/imudiag/imu_diag_ros_device_adis1650x.h"
#include "imu_ros2/imudiag/imu_diag_ros_device_adis1654x.h"
#include "imu_ros2/imudiag/imu_diag_ros_device_adis1655x.h"
#include "imu_ros2/imudiag/imu_diag_ros_device_adis1657x.h"

ImuDiagRosPublisherDevices::ImuDiagRosPublisherDevices(std::shared_ptr<rclcpp::Node> &node) :
  ImuDiagRosPublisherDevicesInterface(node),
  m_diag_device(nullptr)
{
  createPublisher();
}

ImuDiagRosPublisherDevices::~ImuDiagRosPublisherDevices()
{
  delete m_diag_device;
}

void ImuDiagRosPublisherDevices::createPublisher()
{
  if(AdisData::GetInstance()->checkDeviceName("adis1646x"))
    m_diag_device = new ImuDiagRosDevice_adis1646x(m_node);
  if(AdisData::GetInstance()->checkDeviceName("adis1647x"))
    m_diag_device = new ImuDiagRosDevice_adis1647x(m_node);
  if(AdisData::GetInstance()->checkDeviceName("adis1650x"))
    m_diag_device = new ImuDiagRosDevice_adis1650x(m_node);
  if(AdisData::GetInstance()->checkDeviceName("adis1654x"))
    m_diag_device = new ImuDiagRosDevice_adis1654x(m_node);
  if(AdisData::GetInstance()->checkDeviceName("adis1655x"))
    m_diag_device = new ImuDiagRosDevice_adis1655x(m_node);
  if(AdisData::GetInstance()->checkDeviceName("adis1657x"))
    m_diag_device = new ImuDiagRosDevice_adis1657x(m_node);
}

void ImuDiagRosPublisherDevices::publishMessage(ImuDiagDataProviderInterface * dataProvider)
{
  m_diag_device->publishMessage(dataProvider);
}
