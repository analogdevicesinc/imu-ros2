#ifndef GYROSCOPE_ROS_SUBSCRIBER_H
#define GYROSCOPE_ROS_SUBSCRIBER_H

#include "imu_ros2/gyroscope_ros_publisher_interface.h"
#include "imu_ros2/gyroscope_data_provider_interface.h"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class GyroscopeRosPublisher : public GyroscopeRosPublisherInterface {

public:
    GyroscopeRosPublisher(std::shared_ptr<rclcpp::Node>& node);
    ~GyroscopeRosPublisher();

     void init(std::shared_ptr<rclcpp::Node>& node) override;
     void setMessageProvider(GyroscopeDataProviderInterface* dataProvider) override;

     void run() override;

private:

     GyroscopeDataProviderInterface* m_dataProvider;
     rclcpp::Publisher<imu_ros2::msg::GyroscopeData>::SharedPtr m_publisher;
     imu_ros2::msg::GyroscopeData m_message;
};

#endif // GYROSCOPE_ROS_SUBSCRIBER_H
