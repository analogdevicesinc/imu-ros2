/***************************************************************************//**
*   @file   imu_control_subscriber_test.cpp
*   @brief  Test static data
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

#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>
#include <chrono>
#include "imu_ros2/msg/imu_control_data.hpp"

class ImuControlSubscriberTest  : public ::testing::Test
{
public:
    static void SetUpTestCase()
    {

    }

    static void TearDownTestCase()
    {
        rclcpp::shutdown();
    }
};

TEST(ImuControlSubscriberTest,test_imu_control_data_values1)
{
    auto node = rclcpp::Node::make_shared("imucontroldata");

    std::string topic = "imucontroldata";

    int counter = 0;


    auto callback =
            [&counter]( imu_ros2::msg::ImuControlData msg) -> void
    {
        ++counter;

        RCLCPP_INFO(rclcpp::get_logger("rclcpp_imu_control_data"), " device control data info: %d %d %d  \n",
                    msg.anglvel_x_calibbias, msg.burst_size_selection, msg.burst_data_selection );

        // TODO: make assert with values from ImuControlData type
        ASSERT_TRUE(true);
    };

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    auto subscriber = node->create_subscription<imu_ros2::msg::ImuControlData>(topic, 10, callback);

    std::chrono::seconds sec(1);


    for(int i=0;i<100;i++)
    {
    executor.spin_once(sec);
    }

    ASSERT_TRUE(true);
}

