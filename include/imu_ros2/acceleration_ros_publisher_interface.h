#ifndef ACCELERATION_ROS_PUBLISHER_INTERFACE_H
#define ACCELERATION_ROS_PUBLISHER_INTERFACE_H

#include "imu_ros2/ros_task.h"

#include <rclcpp/rclcpp.hpp>
#include <memory>

class AccelerationDataProviderInterface;

class AccelerationRosPublisherInterface : public RosTask {
public:
    AccelerationRosPublisherInterface(){}
    virtual ~AccelerationRosPublisherInterface(){}

    virtual void init(std::shared_ptr<rclcpp::Node>& node) = 0;
    virtual void setMessageProvider(AccelerationDataProviderInterface* dataProvider) = 0;

protected:
    std::shared_ptr<rclcpp::Node> m_node;
};

#endif // ACCELERATION_ROS_PUBLISHER_INTERFACE_H
