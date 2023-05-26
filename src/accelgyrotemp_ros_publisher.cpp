/***************************************************************************//**
*   @file   accelgyrotemp_ros_publisher.cpp
*   @brief  Implementation for accel, gyro and temp publisher
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

#include "imu_ros2/accelgyrotemp_ros_publisher.h"
#include <thread>
#include <chrono>

AccelGyroTempRosPublisher::AccelGyroTempRosPublisher(std::shared_ptr<rclcpp::Node>& node)
{
    init(node);
}

AccelGyroTempRosPublisher::~AccelGyroTempRosPublisher()
{
    delete m_dataProvider;
}

void AccelGyroTempRosPublisher::init(std::shared_ptr<rclcpp::Node> &node)
{
    m_node = node;
    m_publisher = node->create_publisher<imu_ros2::msg::AccelGyroTempData>("accelgyrotempdata", 10);
}

void AccelGyroTempRosPublisher::setMessageProvider(AccelGyroTempDataProviderInterface *dataProvider)
{
    m_dataProvider = dataProvider;
}

void AccelGyroTempRosPublisher::run()
{
    std::thread::id this_id = std::this_thread::get_id();
    std::cout << "thread " << this_id << " started...\n";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp_accelgyrotemp"), "startThread: '%d'", this_id);

    bool success = false;

    rclcpp::WallRate loopRate(0.1);

    while (rclcpp::ok()) {

        int32_t operation_mode = m_node->get_parameter("operation_mode").get_parameter_value().get<int32_t>();

        switch(operation_mode) {
        case DEVICE_CONTINUOUS_SAMPLING_MODE:
            success = false;
            success = m_dataProvider->getData(m_message);

            RCLCPP_INFO(rclcpp::get_logger("rclcpp_accelgyrotemp"), "Publishing acceleration data: '%f' '%f' '%f'",
                        m_message.accel.x, m_message.accel.y, m_message.accel.z);

            if(success)
                m_publisher->publish(m_message);
            break;
        default:
        {
             loopRate.sleep();
             break;
        }
        }
    }
    this_id = std::this_thread::get_id();
    std::cout << "thread " << this_id << " ended...\n";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp_accelgyrotemp"), "endThread: '%d'", this_id);
}
