/***************************************************************************//**
*   @file   imu_control_ros_publisher.h
*   @brief  Publish imu control data on topic
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

#ifndef IMU_CONTROL_ROS_PUBLISHER_H
#define IMU_CONTROL_ROS_PUBLISHER_H

#include "imu_ros2/imu_control_ros_publisher_interface.h"
#include "imu_ros2/imu_control_data_provider_interface.h"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class ImuControlRosPublisher : public ImuControlRosPublisherInterface {

public:
    ImuControlRosPublisher(std::shared_ptr<rclcpp::Node>& node);
    ~ImuControlRosPublisher();

     void init(std::shared_ptr<rclcpp::Node>& node) override;
     void setMessageProvider(ImuControlDataProviderInterface* dataProvider) override;

     void run() override;

private:

     ImuControlDataProviderInterface* m_dataProvider;
     rclcpp::Publisher<imu_ros2::msg::ImuControlData>::SharedPtr m_publisher;
     imu_ros2::msg::ImuControlData m_message;
};

#endif // IMU_CONTROL_ROS_PUBLISHER_H