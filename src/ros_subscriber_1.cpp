/***************************************************************************//**
*   @file   ros_subscriber_1.cpp
*   @brief  Implementation for test publisher
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

#include "imu_ros2/ros_publisher_1.h"
#include <thread>


RosPublisher1::RosPublisher1(std::shared_ptr<rclcpp::Node>& node)
{
    init(node);
}

RosPublisher1::~RosPublisher1()
{
    delete m_dataProvider;
}

void RosPublisher1::init(std::shared_ptr<rclcpp::Node> &node)
{
    m_node = node;
    m_publisher = node->create_publisher<std_msgs::msg::String>("topicStr", 10);
}

void RosPublisher1::setMessageProvider(DataProviderInterface *dataProvider)
{
    m_dataProvider = dataProvider;
}

void RosPublisher1::run()
{
    std::thread::id this_id = std::this_thread::get_id();
    std::cout << "thread " << this_id << " started...\n";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "startThread: '%d'", this_id);

    rclcpp::WallRate loopRate(0.1);

    int count = 0;
    while (rclcpp::ok()) {

        //std::thread::id this_id = std::this_thread::get_id();
        //std::cout << "thread " << this_id << " running...\n";
        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "running: '%d'", this_id);

        std::string my_param =
             m_node->get_parameter("my_parameter").get_parameter_value().get<std::string>();

        m_dataProvider->setParameter(my_param);

        m_message = m_dataProvider->getData(count);
        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Publishing: '%s'", m_message.data.c_str());

        m_publisher->publish(m_message);
        count++;
        //rclcpp::spin_some(m_node);

        //loopRate.sleep();
    }
    this_id = std::this_thread::get_id();
    std::cout << "thread " << this_id << " ended...\n";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "endThread: '%d'", this_id);
}
