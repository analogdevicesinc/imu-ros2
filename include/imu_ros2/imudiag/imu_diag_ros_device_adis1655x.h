#ifndef IMU_DIAG_ROS_DEVICE_ADIS1655X_H
#define IMU_DIAG_ROS_DEVICE_ADIS1655X_H

#include "imu_diag_ros_device_interface.h"
#include "imu_ros2/msg/imu_diag_dataadis1655x.hpp"

class ImuDiagRosDevice_adis1655x : public ImuDiagRosDeviceInterface
{
public:
  /**
   * @brief Constructor for ImuDiagRosDevice_adis1655x.
   */
  ImuDiagRosDevice_adis1655x(std::shared_ptr<rclcpp::Node> & node);

  /**
   * @brief Destructor for ImuDiagRosDevice_adis1655x.
   */
  virtual ~ImuDiagRosDevice_adis1655x();

  void createPublisher() override;

  void publishMessage(ImuDiagDataProviderInterface * dataProvider) override;

protected:

  rclcpp::Publisher<imu_ros2::msg::ImuDiagDataadis1655x>::SharedPtr m_publisher_adis1655x;
  imu_ros2::msg::ImuDiagDataadis1655x m_message_adis1655x;

};

#endif // IMU_DIAG_ROS_DEVICE_ADIS1655X_H
