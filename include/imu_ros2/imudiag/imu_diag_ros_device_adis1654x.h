#ifndef IMU_DIAG_ROS_DEVICE_ADIS1654X_H
#define IMU_DIAG_ROS_DEVICE_ADIS1654X_H

#include "imu_diag_ros_device_interface.h"
#include "imu_ros2/msg/imu_diag_dataadis1654x.hpp"

class ImuDiagRosDevice_adis1654x : public ImuDiagRosDeviceInterface
{
public:
  /**
   * @brief Constructor for ImuDiagRosDevice_adis1654x.
   */
  ImuDiagRosDevice_adis1654x(std::shared_ptr<rclcpp::Node> & node);

  /**
   * @brief Destructor for ImuDiagRosDevice_adis1654x.
   */
  virtual ~ImuDiagRosDevice_adis1654x();

  void createPublisher() override;

  void publishMessage(ImuDiagDataProviderInterface * dataProvider) override;

protected:

  rclcpp::Publisher<imu_ros2::msg::ImuDiagDataadis1654x>::SharedPtr m_publisher_adis1654x;
  imu_ros2::msg::ImuDiagDataadis1654x m_message_adis1654x;

};

#endif // IMU_DIAG_ROS_DEVICE_ADIS1654X_H
