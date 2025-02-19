#ifndef IMU_DIAG_ROS_DEVICE_ADIS1650X_H
#define IMU_DIAG_ROS_DEVICE_ADIS1650X_H

#include "imu_diag_ros_device_interface.h"
#include "imu_ros2/msg/imu_diag_dataadis1650x.hpp"

class ImuDiagRosDevice_adis1650x : public ImuDiagRosDeviceInterface
{
public:
  /**
   * @brief Constructor for ImuDiagRosDevice_adis1650x.
   */
  ImuDiagRosDevice_adis1650x(std::shared_ptr<rclcpp::Node> & node);

  /**
   * @brief Destructor for ImuDiagRosDevice_adis1650x.
   */
  virtual ~ImuDiagRosDevice_adis1650x();

  void createPublisher() override;

  void publishMessage(ImuDiagDataProviderInterface * dataProvider) override;

protected:

  rclcpp::Publisher<imu_ros2::msg::ImuDiagDataadis1650x>::SharedPtr m_publisher_adis1650x;
  imu_ros2::msg::ImuDiagDataadis1650x m_message_adis1650x;

};

#endif // IMU_DIAG_ROS_DEVICE_ADIS1650X_H
