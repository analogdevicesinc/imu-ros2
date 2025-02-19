#ifndef IMU_DIAG_ROS_PUBLISHER_DEVICES_H
#define IMU_DIAG_ROS_PUBLISHER_DEVICES_H

#include "imu_ros2/imudiag/imu_diag_ros_publisher_devices_interface.h"
#include "imu_ros2/imudiag/imu_diag_ros_device_interface.h"

class ImuDiagRosPublisherDevices : public ImuDiagRosPublisherDevicesInterface
{
public:
  /**
   * @brief Constructor for ImuDiagRosPublisherDevices.
   */
  ImuDiagRosPublisherDevices(std::shared_ptr<rclcpp::Node> & node);

  /**
   * @brief Destructor for ImuDiagRosPublisherDevices.
   */
  ~ImuDiagRosPublisherDevices();

  void createPublisher() override;

  void publishMessage(ImuDiagDataProviderInterface * dataProvider) override;

private:

  ImuDiagRosDeviceInterface * m_diag_device;
};

#endif // IMU_DIAG_ROS_PUBLISHER_DEVICES_H
