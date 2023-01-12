#ifndef ROSPUBLISHERINTERFACE_H
#define ROSPUBLISHERINTERFACE_H

#include "imu_ros2/rostask.h"

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

#endif // ROSPUBLISHERINTERFACE_H
