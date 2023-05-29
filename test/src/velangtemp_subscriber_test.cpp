/***************************************************************************//**
*   @file   velangtemp_subscriber_test.cpp
*   @brief  Test vel ang temp publisher
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
#include "imu_ros2/msg/vel_ang_temp_data.hpp"

class VelAngTempSubscriberTest  : public ::testing::Test
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

TEST(VelAngTempSubscriberTest,test_velangtemp_data_intervals)
{
    auto node = rclcpp::Node::make_shared("velangtempdata");

    std::string topic = "velangtempdata";

    int counter = 0;


    auto callback =
            [&counter]( imu_ros2::msg::VelAngTempData msg) -> void
    {
        ++counter;

        RCLCPP_INFO(rclcpp::get_logger("rclcpp_test_adiimu_data"), " delta velocity value : %f %f %f \n",
                    msg.delta_vel.x, msg.delta_vel.y, msg.delta_vel.z );

        RCLCPP_INFO(rclcpp::get_logger("rclcpp_test_adiimu_data"), " delta angle value : %f %f %f \n",
                    msg.delta_angle.x, msg.delta_angle.y, msg.delta_angle.z );

        RCLCPP_INFO(rclcpp::get_logger("rclcpp_test_adiimu_data"), " temperature value : %f \n",
                    msg.temp);

        ASSERT_TRUE(msg.delta_vel.x >= -10000 && msg.delta_vel.x <= 10000);
        ASSERT_TRUE(msg.delta_vel.y >= -10000 && msg.delta_vel.y <= 10000);
        ASSERT_TRUE(msg.delta_vel.z >= -10000 && msg.delta_vel.z <= 10000);

        ASSERT_TRUE(msg.delta_angle.x >= -10000 && msg.delta_angle.x <= 10000);
        ASSERT_TRUE(msg.delta_angle.y >= -10000 && msg.delta_angle.y <= 10000);
        ASSERT_TRUE(msg.delta_angle.z >= -10000 && msg.delta_angle.z <= 10000);

        ASSERT_TRUE(msg.temp >= -100 && msg.temp <= 100);
    };

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    auto subscriber = node->create_subscription<imu_ros2::msg::VelAngTempData>(topic, 10, callback);

    std::chrono::seconds sec(1);


    for(int i=0;i<100;i++)
    {
    executor.spin_once(sec);
    }

    //executor.spin();

    ASSERT_TRUE(true);
}

