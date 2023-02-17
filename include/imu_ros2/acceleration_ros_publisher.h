/***************************************************************************//**
*   @file   acceleration_ros_publisher.h
*   @brief  Publish acceleration on topic
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

#ifndef ACCELERATION_ROS_SUBSCRIBER_H
#define ACCELERATION_ROS_SUBSCRIBER_H

#include "imu_ros2/acceleration_ros_publisher_interface.h"
#include "imu_ros2/acceleration_data_provider_interface.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

class AccelerationRosPublisher : public AccelerationRosPublisherInterface {

public:
    AccelerationRosPublisher(std::shared_ptr<rclcpp::Node>& node);
    ~AccelerationRosPublisher();

     void init(std::shared_ptr<rclcpp::Node>& node) override;
     void setMessageProvider(AccelerationDataProviderInterface* dataProvider) override;

     void run() override;

private:

     AccelerationDataProviderInterface* m_dataProvider;
     rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_publisher;
     sensor_msgs::msg::Imu m_message;
};

#endif // ACCELERATION_ROS_SUBSCRIBER_H
