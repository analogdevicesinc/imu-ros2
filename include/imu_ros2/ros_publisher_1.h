#ifndef ROS_SUBSCRIBER_1_H
#define ROS_SUBSCRIBER_1_H

#include "imu_ros2/ros_publisher_interface.h"
#include "imu_ros2/data_provider_interface.h"

#include <rclcpp/rclcpp.hpp>

class RosPublisher1 : public RosPublisherInterface {

public:
    RosPublisher1(std::shared_ptr<rclcpp::Node>& node);
    ~RosPublisher1();

     void init(std::shared_ptr<rclcpp::Node>& node) override;
     void setMessageProvider(DataProviderInterface* dataProvider) override;

     void run() override;

private:

     DataProviderInterface* m_dataProvider;
     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_publisher;
     std_msgs::msg::String m_message;
};

#endif // ROS_SUBSCRIBER_1_H
