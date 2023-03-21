/***************************************************************************//**
*   @file   imu_diag_ros_publisher.cpp
*   @brief  Implementation for imu diag publisher
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

#include "imu_ros2/imu_diag_ros_publisher.h"
#include <thread>


ImuDiagRosPublisher::ImuDiagRosPublisher(std::shared_ptr<rclcpp::Node>& node)
{
    init(node);
}

ImuDiagRosPublisher::~ImuDiagRosPublisher()
{
    delete m_dataProvider;
}

void ImuDiagRosPublisher::init(std::shared_ptr<rclcpp::Node> &node)
{
    m_node = node;
    m_publisher = node->create_publisher<imu_ros2::msg::ImuDiagData>("imudiagdata", 10);
}

void ImuDiagRosPublisher::setMessageProvider(ImuDiagDataProviderInterface *dataProvider)
{
    m_dataProvider = dataProvider;
}

void ImuDiagRosPublisher::run()
{
    std::thread::id this_id = std::this_thread::get_id();
    std::cout << "thread " << this_id << " started...\n";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "startThread: '%d'", this_id);

    rclcpp::WallRate loopRate(0.01);

    int count = 0;
    while (rclcpp::ok())
    {

      //std::thread::id this_id = std::this_thread::get_id();
       // std::cout << "thread " << this_id << " running...\n";
       // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "running: '%d'", this_id);

        //auto started = std::chrono::high_resolution_clock::now();
        m_message = m_dataProvider->getData(count);
        //auto done = std::chrono::high_resolution_clock::now();

        RCLCPP_INFO(rclcpp::get_logger("rclcpp_imu_diag_data"), "Publishing imu diag data: lost_samples_count = '%d' diag_checksum_error_flag= '%d' flash_counter = '%d' ",
                    m_message.lost_samples_count, m_message.diag_checksum_error_flag, m_message.flash_counter);

        m_publisher->publish(m_message);
        count++;
        //rclcpp::spin_some(m_node);
        loopRate.sleep();
    }
    this_id = std::this_thread::get_id();
    std::cout << "thread " << this_id << " ended...\n";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "endThread: '%d'", this_id);
}
