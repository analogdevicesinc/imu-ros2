/***************************************************************************//**
*   @file   gyroscope_subscriber_test.cpp
*   @brief  Test gyroscope
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
#include "imu_ros2/msg/gyroscope_data.hpp"

class GyroscopeSubscriberTest  : public ::testing::Test
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

TEST(GyroscopeSubscriberTest,test_gyroscope_pozitive_values1)
{
    auto node = rclcpp::Node::make_shared("imugyroscope");

    std::string topic = "imugyroscope";

    int counter = 0;


    auto callback =
            [&counter]( imu_ros2::msg::GyroscopeData msg) -> void
    {
        ++counter;

        RCLCPP_INFO(rclcpp::get_logger("rclcpp_test_gyroscope_data"), " gyroscope value : %f %f %f \n",
                    msg.anglvel_x, msg.anglvel_y, msg.anglvel_z );
        ASSERT_TRUE(msg.anglvel_x >= 0);
        ASSERT_TRUE(msg.anglvel_y >= 0);
        ASSERT_TRUE(msg.anglvel_z >= 0);



    };

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    auto subscriber = node->create_subscription<imu_ros2::msg::GyroscopeData>(topic, 10, callback);

    std::chrono::seconds sec(1);


    for(int i=0;i<100;i++)
    {
    executor.spin_once(sec);
    }

    //executor.spin();

    ASSERT_TRUE(true);
}

