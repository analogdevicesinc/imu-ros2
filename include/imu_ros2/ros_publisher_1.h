/***************************************************************************//**
*   @file   ros_publisher_1.h
*   @brief  Test publisher
*   @author Vasile Holonec (Vasile.Holonec@analog.com)
********************************************************************************
* Copyright 2023(c) Analog Devices, Inc.

* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

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
