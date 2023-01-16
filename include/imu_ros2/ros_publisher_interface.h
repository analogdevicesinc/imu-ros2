#ifndef ROS_PUBLISHER_INTERFACE_H
#define ROS_PUBLISHER_INTERFACE_H

#include "imu_ros2/ros_task.h"

#include <rclcpp/rclcpp.hpp>
#include <memory>

class DataProviderInterface;

class RosPublisherInterface : public RosTask {
public:
    RosPublisherInterface(){}
    virtual ~RosPublisherInterface(){}

    virtual void init(std::shared_ptr<rclcpp::Node>& node) = 0;
    virtual void setMessageProvider(DataProviderInterface* dataProvider) = 0;

protected:
    std::shared_ptr<rclcpp::Node> m_node;
};

#endif // ROS_PUBLISHER_INTERFACE_H
