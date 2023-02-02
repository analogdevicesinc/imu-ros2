#ifndef GYROSCOPE_ROS_PUBLISHER_INTERFACE_H
#define GYROSCOPE_ROS_PUBLISHER_INTERFACE_H

#include "imu_ros2/ros_task.h"

#include <rclcpp/rclcpp.hpp>
#include <memory>

class GyroscopeDataProviderInterface;

class GyroscopeRosPublisherInterface : public RosTask {
public:
    GyroscopeRosPublisherInterface(){}
    virtual ~GyroscopeRosPublisherInterface(){}

    virtual void init(std::shared_ptr<rclcpp::Node>& node) = 0;
    virtual void setMessageProvider(GyroscopeDataProviderInterface* dataProvider) = 0;

protected:
    std::shared_ptr<rclcpp::Node> m_node;
};

#endif // GYROSCOPE_ROS_PUBLISHER_INTERFACE_H
