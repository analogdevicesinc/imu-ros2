#ifndef IMU_DIAG_ROS_DEVICE_ADIS1657X_H
#define IMU_DIAG_ROS_DEVICE_ADIS1657X_H

#include "imu_diag_ros_device_interface.h"
#include "imu_ros2/msg/imu_diag_dataadis1657x.hpp"

class ImuDiagRosDevice_adis1657x : public ImuDiagRosDeviceInterface
{
public:
  /**
   * @brief Constructor for ImuDiagRosDevice_adis1657x.
   */
  ImuDiagRosDevice_adis1657x(std::shared_ptr<rclcpp::Node> & node);

  /**
   * @brief Destructor for ImuDiagRosDevice_adis1657x.
   */
  virtual ~ImuDiagRosDevice_adis1657x();

  void createPublisher() override;

  void publishMessage(ImuDiagDataProviderInterface * dataProvider) override;

protected:

  rclcpp::Publisher<imu_ros2::msg::ImuDiagDataadis1657x>::SharedPtr m_publisher_adis1657x;
  imu_ros2::msg::ImuDiagDataadis1657x m_message_adis1657x;

};

#endif // IMU_DIAG_ROS_DEVICE_ADIS1657X_H
