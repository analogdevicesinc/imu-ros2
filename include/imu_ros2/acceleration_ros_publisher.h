#ifndef ACCELERATION_ROS_SUBSCRIBER_H
#define ACCELERATION_ROS_SUBSCRIBER_H

#include "imu_ros2/acceleration_ros_publisher_interface.h"
#include "imu_ros2/acceleration_data_provider_interface.h"

#include <rclcpp/rclcpp.hpp>

class AccelerationRosPublisher : public AccelerationRosPublisherInterface {

public:
    AccelerationRosPublisher(std::shared_ptr<rclcpp::Node>& node);
    ~AccelerationRosPublisher();

     void init(std::shared_ptr<rclcpp::Node>& node) override;
     void setMessageProvider(AccelerationDataProviderInterface* dataProvider) override;

     void run() override;

private:

     AccelerationDataProviderInterface* m_dataProvider;
     rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr m_publisher;
     std_msgs::msg::UInt32 m_message;
};

#endif // ACCELERATION_ROS_SUBSCRIBER_H
