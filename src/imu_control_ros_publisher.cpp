/***************************************************************************//**
*   @file   imu_control_ros_publisher.cpp
*   @brief  Implementation for imu control publisher
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

#include "imu_ros2/imu_control_ros_publisher.h"
#include <thread>

ImuControlRosPublisher::ImuControlRosPublisher(std::shared_ptr<rclcpp::Node>& node)
{
    init(node);
}

ImuControlRosPublisher::~ImuControlRosPublisher()
{
    delete m_dataProvider;
}

void ImuControlRosPublisher::init(std::shared_ptr<rclcpp::Node> &node)
{
    m_node = node;
    m_publisher = node->create_publisher<imu_ros2::msg::ImuControlData>("imucontroldata", 10);
}

void ImuControlRosPublisher::setMessageProvider(ImuControlDataProviderInterface *dataProvider)
{
    m_dataProvider = dataProvider;
}

void ImuControlRosPublisher::run()
{
    std::thread::id this_id = std::this_thread::get_id();
    std::cout << "thread " << this_id << " started...\n";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "startThread: '%d'", this_id);

    rclcpp::WallRate loopRate(0.1);


    while (rclcpp::ok()) {

        int32_t filter_size =
             m_node->get_parameter("filter_size").get_parameter_value().get<int32_t>();

        m_dataProvider->set_filter_size(filter_size);

        m_message = m_dataProvider->getData();

        m_publisher->publish(m_message);

        loopRate.sleep();
    }
    this_id = std::this_thread::get_id();
    std::cout << "thread " << this_id << " ended...\n";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "endThread: '%d'", this_id);
}
