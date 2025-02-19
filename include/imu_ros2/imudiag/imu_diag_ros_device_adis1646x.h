#ifndef IMU_DIAG_ROS_DEVICE_ADIS1646X_H
#define IMU_DIAG_ROS_DEVICE_ADIS1646X_H

#include "imu_diag_ros_device_interface.h"
#include "imu_ros2/msg/imu_diag_dataadis1646x.hpp"

class ImuDiagRosDevice_adis1646x : public ImuDiagRosDeviceInterface
{
public:
  /**
   * @brief Constructor for ImuDiagRosDevice_adis1646x.
   */
  ImuDiagRosDevice_adis1646x(std::shared_ptr<rclcpp::Node> & node);

  /**
   * @brief Destructor for ImuDiagRosDevice_adis1646x.
   */
  virtual ~ImuDiagRosDevice_adis1646x();

  void createPublisher() override;

  void publishMessage(ImuDiagDataProviderInterface * dataProvider) override;

protected:

  rclcpp::Publisher<imu_ros2::msg::ImuDiagDataadis1646x>::SharedPtr m_publisher_adis1646x;
  imu_ros2::msg::ImuDiagDataadis1646x m_message_adis1646x;

};

#endif // IMU_DIAG_ROS_DEVICE_ADIS1646X_H
