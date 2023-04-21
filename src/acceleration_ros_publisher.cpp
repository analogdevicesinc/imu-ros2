/***************************************************************************//**
*   @file   acceleration_ros_publisher.cpp
*   @brief  Implementation for acceleration publisher
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

#include "imu_ros2/acceleration_ros_publisher.h"
#include <thread>
#include <chrono>

AccelerationRosPublisher::AccelerationRosPublisher(std::shared_ptr<rclcpp::Node>& node)
{
    init(node);
}

AccelerationRosPublisher::~AccelerationRosPublisher()
{
    delete m_dataProvider;
}

void AccelerationRosPublisher::init(std::shared_ptr<rclcpp::Node> &node)
{
    m_node = node;
    m_publisher = node->create_publisher<sensor_msgs::msg::Imu>("imuacceleration", 10);
}

void AccelerationRosPublisher::setMessageProvider(AccelerationDataProviderInterface *dataProvider)
{
    m_dataProvider = dataProvider;
}

void AccelerationRosPublisher::run()
{
    std::thread::id this_id = std::this_thread::get_id();
    std::cout << "thread " << this_id << " started...\n";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "startThread: '%d'", this_id);

    typedef std::chrono::system_clock::time_point timenow;
    timenow started;
    timenow done;
    rclcpp::WallRate loopRate(0.1);
    bool success = false;

    while (rclcpp::ok()) {

        //std::thread::id this_id = std::this_thread::get_id();
        //std::cout << "thread " << this_id << " running...\n";
        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "running: '%d'", this_id);

        int32_t operation_mode = m_node->get_parameter("operation_mode").get_parameter_value().get<int32_t>();

        switch(operation_mode) {
        case DEVICE_CONTINUOUS_SAMPLING_MODE:
            m_dataProvider->load();
            started = std::chrono::high_resolution_clock::now();
            success = false;
            m_message = m_dataProvider->getData(success);
            done = std::chrono::high_resolution_clock::now();

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Publishing acceleration x y z in time %d ms: '%f' '%f' '%f'",
                        m_message.linear_acceleration.x, m_message.linear_acceleration.y, m_message.linear_acceleration.z ,
                        std::chrono::duration_cast<std::chrono::milliseconds>(done-started).count());

            if(success)
                m_publisher->publish(m_message);
            break;
        case USER_PARAM_SETTING_MODE:
            m_dataProvider->unload();
            break;
        default:
        {
            loopRate.sleep();
            break;
        }
        }
        rclcpp::spin_some(m_node);
    }
    this_id = std::this_thread::get_id();
    std::cout << "thread " << this_id << " ended...\n";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "endThread: '%d'", this_id);
}
