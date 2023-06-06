/***************************************************************************//**
*   @file   velangtemp_ros_publisher.cpp
*   @brief  Implementation for vel ang temp publisher
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

#include "imu_ros2/velangtemp_ros_publisher.h"
#include <thread>
#include <chrono>

VelAngTempRosPublisher::VelAngTempRosPublisher(std::shared_ptr<rclcpp::Node>& node)
{
    init(node);
}

VelAngTempRosPublisher::~VelAngTempRosPublisher()
{
    delete m_dataProvider;
}

void VelAngTempRosPublisher::init(std::shared_ptr<rclcpp::Node> &node)
{
    m_node = node;
    m_publisher = node->create_publisher<imu_ros2::msg::VelAngTempData>("velangtempdata", 10);
}

void VelAngTempRosPublisher::setMessageProvider(VelAngTempDataProviderInterface *dataProvider)
{
    m_dataProvider = dataProvider;
}

void VelAngTempRosPublisher::run()
{
    std::thread::id this_id = std::this_thread::get_id();
    std::cout << "thread " << this_id << " started...\n";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "startThread: '%d'", this_id);

    bool bufferedDataEnabled  = false;
    int32_t bufferedDataSelection = 0;

    rclcpp::WallRate loopRate(0.1);

    while (rclcpp::ok()) {

        int32_t operation_mode = m_node->get_parameter("operation_mode").get_parameter_value().get<int32_t>();

        switch(operation_mode) {
        case DEVICE_CONTINUOUS_SAMPLING_MODE:

            bufferedDataSelection = m_node->get_parameter("buffered_data_selection").get_parameter_value().get<int32_t>();

            switch(bufferedDataSelection) {
            case DELTAVEL_DELTAANG_BUFFERED_DATA:
                if(!bufferedDataEnabled)
                {
                    if(m_dataProvider->enableBufferedDataOutput())
                        bufferedDataEnabled  = true;
                    else
                        loopRate.sleep();
                }
                else
                {
                    if(m_dataProvider->getData(m_message))
                    {
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp_velangtemp"), "Publishing delta velocity data: '%f' '%f' '%f'",
                                    m_message.delta_vel.x, m_message.delta_vel.y, m_message.delta_vel.z);
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp_velangtemp"), "Publishing delta angle data: '%f' '%f' '%f'",
                                    m_message.delta_angle.x, m_message.delta_angle.y, m_message.delta_angle.z);
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp_velangtemp"), "Publishing temperature data: '%f' ",
                                    m_message.temp);

                        m_publisher->publish(m_message);
                    }
                }
                break;

            case ACCEL_GYRO_BUFFERED_DATA:
                bufferedDataEnabled = false;
                loopRate.sleep();
                break;
            default:
            {
                loopRate.sleep();
                break;
            }
            }
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
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "endThread: '%d'", this_id);
}
