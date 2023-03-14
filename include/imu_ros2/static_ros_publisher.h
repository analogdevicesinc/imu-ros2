/***************************************************************************//**
*   @file   static_ros_publisher.h
*   @brief  Publish static data on topic
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
     rclcpp::Publisher<imu_ros2::msg::ImuIdentificationData>::SharedPtr m_publisher;
     imu_ros2::msg::ImuIdentificationData m_message;
};

#endif // STATIC_ROS_SUBSCRIBER_H
