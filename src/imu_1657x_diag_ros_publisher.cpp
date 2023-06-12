/***************************************************************************//**
*   @file   imu_1657x_diag_ros_publisher.cpp
*   @brief  Implementation for adis1657x imu diag publisher
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

#include "imu_ros2/imu_1657x_diag_ros_publisher.h"
#include <thread>


Imu1657xDiagRosPublisher::Imu1657xDiagRosPublisher(std::shared_ptr<rclcpp::Node>& node)
{
    init(node);
}

Imu1657xDiagRosPublisher::~Imu1657xDiagRosPublisher()
{
    delete m_dataProvider;
}

void Imu1657xDiagRosPublisher::init(std::shared_ptr<rclcpp::Node> &node)
{
    m_node = node;
    m_publisher = node->create_publisher<imu_ros2::msg::Imu1657xDiagData>("imu1657xdiagdata", 10);
}

void Imu1657xDiagRosPublisher::setMessageProvider(Imu1657xDiagDataProviderInterface *dataProvider)
{
    m_dataProvider = dataProvider;
}

void Imu1657xDiagRosPublisher::run()
{
    std::thread::id this_id = std::this_thread::get_id();
    std::cout << "thread " << this_id << " started...\n";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "startThread: '%d'", this_id);

    rclcpp::WallRate loopRate(0.01);

    while (rclcpp::ok())
    {
        int32_t operation_mode = m_node->get_parameter("operation_mode").get_parameter_value().get<int32_t>();

        switch(operation_mode) {
        case DEVICE_CONTINUOUS_SAMPLING_MODE:
            if(m_dataProvider->getData(m_message))
            {
                m_publisher->publish(m_message);
            }
            break;
        default:
        {
            break;
        }
        }
        loopRate.sleep();
    }
    this_id = std::this_thread::get_id();
    std::cout << "thread " << this_id << " ended...\n";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "endThread: '%d'", this_id);
}
