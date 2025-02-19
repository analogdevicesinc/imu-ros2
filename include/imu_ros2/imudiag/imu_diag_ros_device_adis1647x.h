#ifndef IMU_DIAG_ROS_DEVICE_ADIS1647X_H
#define IMU_DIAG_ROS_DEVICE_ADIS1647X_H

#include "imu_diag_ros_device_interface.h"
#include "imu_ros2/msg/imu_diag_dataadis1647x.hpp"

class ImuDiagRosDevice_adis1647x : public ImuDiagRosDeviceInterface
{
public:
  /**
   * @brief Constructor for ImuDiagRosDevice_adis1647x.
   */
  ImuDiagRosDevice_adis1647x(std::shared_ptr<rclcpp::Node> & node);

  /**
   * @brief Destructor for ImuDiagRosDevice_adis1647x.
   */
  virtual ~ImuDiagRosDevice_adis1647x();

  void createPublisher() override;

  void publishMessage(ImuDiagDataProviderInterface * dataProvider) override;

protected:

  rclcpp::Publisher<imu_ros2::msg::ImuDiagDataadis1647x>::SharedPtr m_publisher_adis1647x;
  imu_ros2::msg::ImuDiagDataadis1647x m_message_adis1647x;

};

#endif // IMU_DIAG_ROS_DEVICE_ADIS1647X_H
