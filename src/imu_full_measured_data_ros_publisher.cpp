/***************************************************************************//**
*   @file   imu_full_measured_data_ros_publisher.cpp
*   @brief  Implementation for raw adi imu publisher
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

#include "imu_ros2/imu_full_measured_data_ros_publisher.h"
#include <thread>
#include <chrono>

ImuFullMeasuredDataRosPublisher::ImuFullMeasuredDataRosPublisher(std::shared_ptr<rclcpp::Node>& node)
{
    init(node);
}

ImuFullMeasuredDataRosPublisher::~ImuFullMeasuredDataRosPublisher()
{
    delete m_dataProvider;
}

void ImuFullMeasuredDataRosPublisher::init(std::shared_ptr<rclcpp::Node> &node)
{
    m_node = node;
    m_publisher = node->create_publisher<imu_ros2::msg::ImuFullMeasuredData>("imufullmeasureddata", 10);
}

void ImuFullMeasuredDataRosPublisher::setMessageProvider(ImuFullMeasuredDataProviderInterface *dataProvider)
{
    m_dataProvider = dataProvider;
}

void ImuFullMeasuredDataRosPublisher::run()
{
    std::thread::id this_id = std::this_thread::get_id();
    std::cout << "thread " << this_id << " started...\n";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "startThread: '%d'", this_id);

    typedef std::chrono::system_clock::time_point timenow;
    timenow started;
    timenow done;
    bool success = false;

    rclcpp::WallRate loopRate(0.1);

    while (rclcpp::ok()) {

        int32_t operation_mode = m_node->get_parameter("operation_mode").get_parameter_value().get<int32_t>();

        switch(operation_mode) {
        case DEVICE_CONTINUOUS_SAMPLING_MODE:
            started = std::chrono::high_resolution_clock::now();
            success = m_dataProvider->getData(m_message);
            done = std::chrono::high_resolution_clock::now();

            if(success)
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp_imu_full_measured_data"), "Publishing imu acceleration data: '%f' '%f' '%f'",
                            m_message.accel.x, m_message.accel.y, m_message.accel.z);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp_imu_full_measured_data"), "Publishing imu gyroscope data: '%f' '%f' '%f'",
                            m_message.gyro.x, m_message.gyro.y, m_message.gyro.z);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp_imu_full_measured_data"), "Publishing imu delta velocity data: '%f' '%f' '%f'",
                            m_message.delta_vel.x, m_message.delta_vel.y, m_message.delta_vel.z);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp_imu_full_measured_data"), "Publishing imu delta angle data: '%f' '%f' '%f'",
                            m_message.delta_angle.x, m_message.delta_angle.y, m_message.delta_angle.z);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp_imu_full_measured_data"), "Publishing imu temperature data: '%f' ",
                            m_message.temp);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp_imu_full_measured_data"), "Publishing imu time %d us ",
                            std::chrono::duration_cast<std::chrono::microseconds>(done-started).count());

                    m_publisher->publish(m_message);
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
