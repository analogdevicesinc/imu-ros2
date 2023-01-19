#ifndef STATIC_ROS_PUBLISHER_INTERFACE_H
#define STATIC_ROS_PUBLISHER_INTERFACE_H

#include "imu_ros2/ros_task.h"

#include <rclcpp/rclcpp.hpp>
#include <memory>

class StaticDataProviderInterface;

class StaticRosPublisherInterface : public RosTask {
public:
    StaticRosPublisherInterface(){}
    virtual ~StaticRosPublisherInterface(){}

    virtual void init(std::shared_ptr<rclcpp::Node>& node) = 0;
    virtual void setMessageProvider(StaticDataProviderInterface* dataProvider) = 0;

protected:
    std::shared_ptr<rclcpp::Node> m_node;
};

#endif // STATIC_ROS_PUBLISHER_INTERFACE_H
