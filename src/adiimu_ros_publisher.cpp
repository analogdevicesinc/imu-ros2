/***************************************************************************//**
*   @file   adiimu_ros_publisher.cpp
*   @brief  Implementation for adi imu publisher
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

#include "imu_ros2/adiimu_ros_publisher.h"
#include <thread>
#include <chrono>

AdiImuRosPublisher::AdiImuRosPublisher(std::shared_ptr<rclcpp::Node>& node)
{
    init(node);
}

AdiImuRosPublisher::~AdiImuRosPublisher()
{
    delete m_dataProvider;
}

void AdiImuRosPublisher::init(std::shared_ptr<rclcpp::Node> &node)
{
    m_node = node;
    m_publisher = node->create_publisher<imu_ros2::msg::AdiImuData>("adiimudata", 10);
}

void AdiImuRosPublisher::setMessageProvider(AdiImuDataProviderInterface *dataProvider)
{
    m_dataProvider = dataProvider;
}

void AdiImuRosPublisher::run()
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

        auto started = std::chrono::high_resolution_clock::now();
        m_message = m_dataProvider->getData(count);
        auto done = std::chrono::high_resolution_clock::now();

        RCLCPP_INFO(rclcpp::get_logger("rclcpp_adiimu"), "Publishing adi imu acceleration data: '%f' '%f' '%f'",
                    m_message.accel.x, m_message.accel.y, m_message.accel.z);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp_adiimu"), "Publishing adi imu gyroscope data: '%f' '%f' '%f'",
                    m_message.gyro.x, m_message.gyro.y, m_message.gyro.z);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp_adiimu"), "Publishing adi imu delta velocity data: '%f' '%f' '%f'",
                    m_message.delta_vel.x, m_message.delta_vel.y, m_message.delta_vel.z);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp_adiimu"), "Publishing adi imu delta angle data: '%f' '%f' '%f'",
                    m_message.delta_angle.x, m_message.delta_angle.y, m_message.delta_angle.z);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp_adiimu"), "Publishing adi imu temperature data: '%f' ",
                    m_message.temp);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp_adiimu"), "Publishing adi imu time %d us ",
                    std::chrono::duration_cast<std::chrono::microseconds>(done-started).count());

        m_publisher->publish(m_message);
        count++;
        //rclcpp::spin_some(m_node);
        //loopRate.sleep();
    }
    this_id = std::this_thread::get_id();
    std::cout << "thread " << this_id << " ended...\n";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "endThread: '%d'", this_id);
}
