#ifndef STATIC_ROS_SUBSCRIBER_H
#define STATIC_ROS_SUBSCRIBER_H

#include "imu_ros2/static_ros_publisher_interface.h"
#include "imu_ros2/static_data_provider_interface.h"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class StaticRosPublisher : public StaticRosPublisherInterface {

public:
    StaticRosPublisher(std::shared_ptr<rclcpp::Node>& node);
    ~StaticRosPublisher();

     void init(std::shared_ptr<rclcpp::Node>& node) override;
     void setMessageProvider(StaticDataProviderInterface* dataProvider) override;

     void run() override;

private:

     StaticDataProviderInterface* m_dataProvider;
     rclcpp::Publisher<imu_ros2::msg::StaticData>::SharedPtr m_publisher;
     imu_ros2::msg::StaticData m_message;
};

#endif // STATIC_ROS_SUBSCRIBER_H
