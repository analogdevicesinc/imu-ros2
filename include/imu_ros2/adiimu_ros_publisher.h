/***************************************************************************//**
*   @file   adiimu_ros_publisher.h
*   @brief  Publish adi imu data on topic
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

#ifndef ADIIMU_ROS_SUBSCRIBER_H
#define ADIIMU_ROS_SUBSCRIBER_H

#include "imu_ros2/adiimu_ros_publisher_interface.h"
#include "imu_ros2/adiimu_data_provider_interface.h"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class AdiImuRosPublisher : public AdiImuRosPublisherInterface {

public:
    AdiImuRosPublisher(std::shared_ptr<rclcpp::Node>& node);
    ~AdiImuRosPublisher();

     void init(std::shared_ptr<rclcpp::Node>& node) override;
     void setMessageProvider(AdiImuDataProviderInterface* dataProvider) override;

     void run() override;

private:

     AdiImuDataProviderInterface* m_dataProvider;
     rclcpp::Publisher<imu_ros2::msg::AdiImuData>::SharedPtr m_publisher;
     imu_ros2::msg::AdiImuData m_message;
};

#endif // ADIIMU_ROS_PUBLISHER_H
