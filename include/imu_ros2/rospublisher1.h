#ifndef ROSSUBSCRIBER1_H
#define ROSSUBSCRIBER1_H

#include "imu_ros2/rospublisherinterface.h"
#include "imu_ros2/dataproviderinterface.h"

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

#endif // ROSSUBSCRIBER1_H
